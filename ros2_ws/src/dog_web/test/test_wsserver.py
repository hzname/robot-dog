"""End-to-end test of the stdlib WebSocket server with a raw socket client."""

import asyncio
import base64
import os
import struct

from dog_web.wsserver import (
    OP_CLOSE, OP_PING, OP_PONG, OP_TEXT, Server, WebSocketClosed, accept_key, encode_frame)


def test_accept_key_matches_rfc6455_example():
    assert accept_key('dGhlIHNhbXBsZSBub25jZQ==') == 's3pPLMBiTxaQ9kYGzzhZRbK+xOo='


async def _read_frame(reader):
    b1, b2 = await reader.readexactly(2)
    n = b2 & 0x7F
    if n == 126:
        (n,) = struct.unpack('!H', await reader.readexactly(2))
    return b1 & 0x0F, await reader.readexactly(n)


async def _scenario(tmp_path):
    (tmp_path / 'index.html').write_text('<h1>dog</h1>')
    received = []

    async def on_client(ws):
        try:
            while True:
                text = await ws.recv()
                received.append(text)
                await ws.send('echo:' + text)
        except WebSocketClosed:
            pass

    server = Server(str(tmp_path), on_client)
    port = await server.start('127.0.0.1', 0)
    try:
        # Static file + path traversal protection.
        for path, expect in (('/', b'200 OK'), ('/../etc/passwd', b'404'), ('/nope.js', b'404')):
            r, w = await asyncio.open_connection('127.0.0.1', port)
            w.write(f'GET {path} HTTP/1.1\r\nHost: x\r\n\r\n'.encode())
            await w.drain()
            data = await r.read()
            assert expect in data.split(b'\r\n', 1)[0], (path, data[:60])
            if path == '/':
                assert data.endswith(b'<h1>dog</h1>')
            w.close()

        # WebSocket handshake.
        key = base64.b64encode(os.urandom(16)).decode()
        r, w = await asyncio.open_connection('127.0.0.1', port)
        w.write((
            'GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
        await w.drain()
        head = await r.readuntil(b'\r\n\r\n')
        assert b'101 Switching Protocols' in head
        assert accept_key(key).encode() in head

        mask = b'\x01\x02\x03\x04'
        # Text round trip, including a 16-bit length frame.
        for text in ('{"type":"stop"}', 'x' * 300):
            w.write(encode_frame(OP_TEXT, text.encode(), mask))
            await w.drain()
            op, payload = await _read_frame(r)
            assert op == OP_TEXT and payload.decode() == 'echo:' + text

        # Fragmented message: "ab" + "cd".
        w.write(bytes([0x01, 0x82]) + mask + bytes(b ^ mask[i] for i, b in enumerate(b'ab')))
        w.write(bytes([0x80, 0x82]) + mask + bytes(b ^ mask[i] for i, b in enumerate(b'cd')))
        await w.drain()
        op, payload = await _read_frame(r)
        assert payload == b'echo:abcd'

        # Ping -> pong.
        w.write(encode_frame(OP_PING, b'hi', mask))
        await w.drain()
        op, payload = await _read_frame(r)
        assert (op, payload) == (OP_PONG, b'hi')

        # Close handshake.
        w.write(encode_frame(OP_CLOSE, struct.pack('!H', 1000), mask))
        await w.drain()
        op, _ = await _read_frame(r)
        assert op == OP_CLOSE
        w.close()
        await asyncio.sleep(0.05)
        assert received == ['{"type":"stop"}', 'x' * 300, 'abcd']

        # Unmasked client frames are a protocol error: server hangs up.
        r, w = await asyncio.open_connection('127.0.0.1', port)
        w.write((
            'GET /ws HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\n\r\n').encode())
        await r.readuntil(b'\r\n\r\n')
        w.write(encode_frame(OP_TEXT, b'unmasked'))
        await w.drain()
        rest = await asyncio.wait_for(r.read(), 2.0)
        assert rest == b'' or rest[0] & 0x0F == OP_CLOSE
        w.close()
    finally:
        await server.stop()


def test_server_end_to_end(tmp_path):
    asyncio.run(_scenario(tmp_path))


# --- TEL-03: Origin, client cap, idle sessions ---------------------------------

KEY = base64.b64encode(os.urandom(16)).decode()


def _ws_request(port, origin=None, key=KEY):
    host = '127.0.0.1:%d' % port
    lines = [
        'GET /ws HTTP/1.1',
        'Upgrade: websocket',
        'Connection: Upgrade',
        'Host: ' + host,
        'Sec-WebSocket-Key: ' + key,
    ]
    if origin is not None:
        lines.append('Origin: ' + origin)
    return ('\r\n'.join(lines) + '\r\n\r\n').encode()


async def _echo_client(ws):
    try:
        while True:
            text = await ws.recv()
            await ws.send('echo:' + text)
    except WebSocketClosed:
        return


async def _status_code(port, request):
    r, w = await asyncio.open_connection('127.0.0.1', port)
    try:
        w.write(request)
        await w.drain()
        head = await asyncio.wait_for(r.readuntil(b'\r\n\r\n'), 2.0)
        return head.split(b'\r\n')[0].decode('latin-1')
    finally:
        w.close()


def _scenario_origin(tmp_path):
    async def run():
        server = Server(str(tmp_path), _echo_client, idle_timeout=0)
        port = await server.start('127.0.0.1', 0)
        try:
            # A page served by another host must not talk to the robot.
            assert '403' in await _status_code(
                port, _ws_request(port, origin='http://evil.example:8080'))
            # The page that this server served passes.
            assert '101' in await _status_code(
                port, _ws_request(port, origin='http://127.0.0.1:%d' % port))
            # No Origin header at all: not a browser, allowed (the calibration tool).
            assert '101' in await _status_code(port, _ws_request(port))
        finally:
            await server.stop()
    asyncio.run(run())


def test_origin_must_match_the_host_the_page_came_from(tmp_path):
    _scenario_origin(tmp_path)


def _scenario_cap(tmp_path):
    async def run():
        server = Server(str(tmp_path), _echo_client, max_clients=1, idle_timeout=0)
        port = await server.start('127.0.0.1', 0)
        try:
            r1, w1 = await asyncio.open_connection('127.0.0.1', port)
            w1.write(_ws_request(port))
            await w1.drain()
            head = await r1.readuntil(b'\r\n\r\n')
            assert b'101' in head
            # The first session is live, so the second one is refused.
            assert '503' in await _status_code(port, _ws_request(port))
            w1.close()
            await asyncio.sleep(0.1)
            assert '101' in await _status_code(port, _ws_request(port))
        finally:
            await server.stop()
    asyncio.run(run())


def test_only_so_many_sessions_at_once(tmp_path):
    _scenario_cap(tmp_path)


def _scenario_idle(tmp_path):
    async def run():
        server = Server(str(tmp_path), _echo_client, idle_timeout=0.2)
        port = await server.start('127.0.0.1', 0)
        try:
            r, w = await asyncio.open_connection('127.0.0.1', port)
            w.write(_ws_request(port))
            await w.drain()
            await r.readuntil(b'\r\n\r\n')
            # Say nothing: the server hangs up rather than keeping the socket forever.
            rest = await asyncio.wait_for(r.read(64), 2.0)
            assert rest == b'' or rest[0] & 0x0F == OP_CLOSE
            w.close()
        finally:
            await server.stop()
    asyncio.run(run())


def test_a_silent_session_is_closed(tmp_path):
    _scenario_idle(tmp_path)
