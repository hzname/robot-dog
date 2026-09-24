"""Minimal asyncio HTTP + WebSocket (RFC 6455) server, standard library only.

Only what the teleop page needs: static files over HTTP and text messages
over one WebSocket endpoint. Keeping it dependency-free means the same code
runs on every ROS distro image without extra pip / apt packages.
"""

import asyncio
import base64
import hashlib
import mimetypes
import os
import struct

WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
MAX_MESSAGE = 64 * 1024

OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA


class WebSocketClosed(Exception):
    pass


def accept_key(client_key: str) -> str:
    digest = hashlib.sha1((client_key + WS_GUID).encode('ascii')).digest()
    return base64.b64encode(digest).decode('ascii')


def encode_frame(opcode: int, payload: bytes, mask: bytes = None) -> bytes:
    """Encode one final frame. Servers send unmasked; `mask` is for tests."""
    header = bytearray([0x80 | opcode])
    mask_bit = 0x80 if mask else 0
    n = len(payload)
    if n < 126:
        header.append(mask_bit | n)
    elif n < 65536:
        header.append(mask_bit | 126)
        header += struct.pack('!H', n)
    else:
        header.append(mask_bit | 127)
        header += struct.pack('!Q', n)
    if mask:
        header += mask
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return bytes(header) + payload


class WebSocket:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter, peer: str):
        self.reader = reader
        self.writer = writer
        self.peer = peer
        self.closed = False
        self._send_lock = asyncio.Lock()

    async def _read_frame(self):
        b1, b2 = await self.reader.readexactly(2)
        fin = bool(b1 & 0x80)
        opcode = b1 & 0x0F
        masked = bool(b2 & 0x80)
        length = b2 & 0x7F
        if length == 126:
            (length,) = struct.unpack('!H', await self.reader.readexactly(2))
        elif length == 127:
            (length,) = struct.unpack('!Q', await self.reader.readexactly(8))
        if length > MAX_MESSAGE:
            raise WebSocketClosed('frame too large')
        if not masked:
            raise WebSocketClosed('client frames must be masked')
        mask = await self.reader.readexactly(4)
        data = await self.reader.readexactly(length)
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        return fin, opcode, payload

    async def recv(self) -> str:
        """Next text message. Raises WebSocketClosed when the peer goes away."""
        buffer = bytearray()
        message_opcode = None
        try:
            while True:
                fin, opcode, payload = await self._read_frame()
                if opcode == OP_CLOSE:
                    await self.close()
                    raise WebSocketClosed('closed by peer')
                if opcode == OP_PING:
                    await self._send(OP_PONG, payload)
                    continue
                if opcode == OP_PONG:
                    continue
                if opcode in (OP_TEXT, OP_BINARY):
                    message_opcode = opcode
                    buffer = bytearray(payload)
                elif opcode == OP_CONT and message_opcode is not None:
                    buffer += payload
                else:
                    raise WebSocketClosed('protocol error')
                if len(buffer) > MAX_MESSAGE:
                    raise WebSocketClosed('message too large')
                if fin:
                    if message_opcode == OP_TEXT:
                        return buffer.decode('utf-8', errors='replace')
                    buffer = bytearray()  # binary messages are ignored
                    message_opcode = None
        except (asyncio.IncompleteReadError, ConnectionError) as exc:
            self.closed = True
            raise WebSocketClosed(str(exc)) from exc

    async def _send(self, opcode: int, payload: bytes):
        if self.closed:
            raise WebSocketClosed('already closed')
        async with self._send_lock:
            try:
                self.writer.write(encode_frame(opcode, payload))
                await self.writer.drain()
            except ConnectionError as exc:
                self.closed = True
                raise WebSocketClosed(str(exc)) from exc

    async def send(self, text: str):
        await self._send(OP_TEXT, text.encode('utf-8'))

    async def close(self):
        if not self.closed:
            try:
                await self._send(OP_CLOSE, struct.pack('!H', 1000))
            except WebSocketClosed:
                pass
            self.closed = True
        try:
            self.writer.close()
        except Exception:  # noqa: BLE001 - best effort
            pass


async def _read_request(reader: asyncio.StreamReader):
    data = await reader.readuntil(b'\r\n\r\n')
    if len(data) > 16384:
        raise ValueError('request too large')
    lines = data.decode('latin-1').split('\r\n')
    method, path, _version = lines[0].split(' ', 2)
    headers = {}
    for line in lines[1:]:
        if ':' in line:
            k, v = line.split(':', 1)
            headers[k.strip().lower()] = v.strip()
    return method, path, headers


def _http_response(status: str, body: bytes, content_type: str = 'text/plain; charset=utf-8'):
    head = (
        f'HTTP/1.1 {status}\r\n'
        f'Content-Type: {content_type}\r\n'
        f'Content-Length: {len(body)}\r\n'
        'Cache-Control: no-cache\r\n'
        'Connection: close\r\n\r\n'
    )
    return head.encode('latin-1') + body


class Server:
    """Serves files from `static_dir` and hands WebSocket clients on
    `ws_path` to `on_client(ws)` (a coroutine)."""

    def __init__(self, static_dir: str, on_client, ws_path: str = '/ws'):
        self.static_dir = os.path.abspath(static_dir)
        self.on_client = on_client
        self.ws_path = ws_path
        self._server = None

    async def start(self, host: str, port: int):
        self._server = await asyncio.start_server(self._handle, host, port)
        return self._server.sockets[0].getsockname()[1]

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()

    def _static_file(self, url_path: str):
        path = url_path.split('?', 1)[0]
        if path == '/':
            path = '/index.html'
        full = os.path.abspath(os.path.join(self.static_dir, path.lstrip('/')))
        if not full.startswith(self.static_dir + os.sep) or not os.path.isfile(full):
            return None
        return full

    async def _handle(self, reader, writer):
        peer = '%s:%s' % writer.get_extra_info('peername')[:2]
        try:
            method, path, headers = await asyncio.wait_for(_read_request(reader), 10.0)
        except Exception:  # noqa: BLE001 - malformed / slow clients are dropped
            writer.close()
            return

        is_upgrade = headers.get('upgrade', '').lower() == 'websocket'
        if path.split('?', 1)[0] == self.ws_path and is_upgrade:
            key = headers.get('sec-websocket-key')
            if not key:
                writer.write(_http_response('400 Bad Request', b'missing key'))
                writer.close()
                return
            writer.write((
                'HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\n'
                'Connection: Upgrade\r\n'
                f'Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n'
            ).encode('latin-1'))
            await writer.drain()
            ws = WebSocket(reader, writer, peer)
            try:
                await self.on_client(ws)
            finally:
                await ws.close()
            return

        if method != 'GET':
            writer.write(_http_response('405 Method Not Allowed', b'GET only'))
        else:
            full = self._static_file(path)
            if full is None:
                writer.write(_http_response('404 Not Found', b'not found'))
            else:
                ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
                if ctype.startswith('text/') or ctype == 'application/javascript':
                    ctype += '; charset=utf-8'
                with open(full, 'rb') as f:
                    writer.write(_http_response('200 OK', f.read(), ctype))
        try:
            await writer.drain()
        finally:
            writer.close()
