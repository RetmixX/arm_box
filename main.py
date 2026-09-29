import asyncio
from contextlib import asynccontextmanager
from typing import List, Callable, Any, Literal

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, HTTPException
from pydantic import BaseModel
import json

from arm_function import RobotARM

#positions = [2048, 2048, 2048, 1024, 512, 750]


class TorquePayload(BaseModel):
    enabled: bool


# ---------- Приложение ----------
@asynccontextmanager
async def lifespan(app: FastAPI):
    arm = await asyncio.to_thread(RobotARM, device="COM5", speed=3)
    app.state.arm = arm
    # Шина Dynamixel полудуплексная: любые обращения к ней (и HTTP, и WS)
    # должны идти строго по очереди, поэтому один общий замок.
    app.state.arm_lock = asyncio.Lock()
    yield


app = FastAPI(title="Motors API", lifespan=lifespan)

async def run_on_arm(app: FastAPI, func: Callable[..., Any], *args: Any) -> Any:
    """Выполнить блокирующий вызов руки в потоке под общим замком."""
    async with app.state.arm_lock:
        return await asyncio.to_thread(func, *args)


# ---------- Модель тела запроса ----------
class PositionSet(BaseModel):
    x: int
    z: int
    angel: int          # угол
    pov_gripper: int    # поворот гриппера
    gripper_state: bool # состояние захвата


class WsCommand(PositionSet):
    command: Literal["exec"]
# ---------- GET-маршруты ----------
@app.get("/api/motors/temp")
async def get_temp(request: Request):
    arm: RobotARM = request.app.state.arm
    try:
        return await run_on_arm(request.app, arm.get_motors_temp)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))


@app.get("/api/motors/load")
async def get_load(request: Request):
    arm: RobotARM = request.app.state.arm
    try:
        return await run_on_arm(request.app, arm.get_motors_load)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))


@app.get("/api/motors/pos")
async def get_pos(request: Request):
    arm: RobotARM = request.app.state.arm
    try:
        return await run_on_arm(request.app, arm.get_motors_pos)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))


# ---------- POST-маршрут ----------
@app.post("/api/motors/pos/set")
async def set_pos(request: Request, payload: PositionSet):
    arm: RobotARM = request.app.state.arm
    try:
        await run_on_arm(request.app, arm.move_arm, payload.x, payload.z, payload.angel, payload.pov_gripper, payload.gripper_state)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    return {"status": "Ok"}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    print("client connected: ", ws.client)
    arm: RobotARM = ws.app.state.arm

    try:
        while True:
            raw = await ws.receive_text()

            try:
                cmd = WsCommand.model_validate(json.loads(raw))
            except json.JSONDecodeError as e:
                await ws.send_json({"status": "error", "error": f"bad request: {e}"})
                continue

            try:
                await run_on_arm(ws.app, arm.move_arm, cmd.x, cmd.z, cmd.angel, cmd.pov_gripper, cmd.gripper_state)
            except Exception as e:
                print("move failed: ", e)
                await ws.send_json({"status": "error", "message": str(e)})
                continue

            await ws.send_json({"status": "ok", "message": "move completed"})

    except WebSocketDisconnect:
        print("client disconnected: ", ws.client)