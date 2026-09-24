"""RobotClient against the robot's real WebSocket server code (dog_web.wsserver)."""

import asyncio
import json
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'ros2_ws', 'src', 'dog_web'))

from dog_web.wsserver import Server, WebSocketClosed  # noqa: E402

from robotdog_autocal.client import RobotClient  # noqa: E402


def test_client_round_trip(tmp_path):
    received = []
    ready = threading.Event()
    port_box = {}

    async def on_client(ws):
        try:
            while True:
                msg = json.loads(await ws.recv())
                received.append(msg)
                if msg['type'] == 'cal_hello':
                    await ws.send(json.dumps({'type': 'state', 'mode': 'passive'}))
                    await ws.send(json.dumps({'type': 'cal_info', 'joints': ['a'], 'calibration': {},
                                              'geometry': {}}))
                    await ws.send(json.dumps({'type': 'cal_status', 'pulses': {'a': 1370.0},
                                              'positions': {}, 'power': None}))
                elif msg['type'] == 'cal_pose':
                    await ws.send(json.dumps({'type': 'cal_pose_ok'}))
                elif msg['type'] == 'cal_set':
                    ok = all(isinstance(v, (int, float)) for v in msg['params'].values())
                    await ws.send(json.dumps({'type': 'cal_set_result', 'ok': ok, 'reason': '' if ok else 'bad'}))
        except WebSocketClosed:
            pass

    def serve():
        async def main():
            srv = Server(str(tmp_path), on_client)
            port_box['port'] = await srv.start('127.0.0.1', 0)
            ready.set()
            await asyncio.sleep(5)
            await srv.stop()
        asyncio.run(main())

    threading.Thread(target=serve, daemon=True).start()
    assert ready.wait(5)
    c = RobotClient('127.0.0.1', port_box['port'])
    info = c.info()
    assert info['joints'] == ['a']
    assert c.status()['pulses']['a'] == 1370.0
    c.pose({'a': 12.5})
    assert c.set_params({'a.offset_deg': 3.0}) == (True, '')
    assert c.set_params({'a.offset_deg': 'x'})[0] is False
    c.close()
    assert [m['type'] for m in received] == ['cal_hello', 'cal_pose', 'cal_set', 'cal_set']
    assert received[1]['joints'] == {'a': 12.5}
