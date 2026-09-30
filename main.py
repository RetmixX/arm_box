import asyncio
from contextlib import asynccontextmanager
from typing import List, Callable, Any, Literal
from fastapi.middleware.cors import CORSMiddleware

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, HTTPException
from pydantic import BaseModel
import json
import os
from dotenv import load_dotenv
from arm_function import RobotARM, ARMIKError

load_dotenv()

device_path = os.getenv("DEVICE_PATH")

if device_path is None:
    print("Param DEVICE_PATH not found in .env")
    exit(1)

class TorquePayload(BaseModel):
    enabled: bool


# ---------- Приложение ----------
@asynccontextmanager
async def lifespan(app: FastAPI):
    arm = await asyncio.to_thread(RobotARM, device=device_path, speed=3)
    app.state.arm = arm
    app.state.arm_lock = asyncio.Lock()
    yield


app = FastAPI(title="Motors API", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
async def run_on_arm(app: FastAPI, func: Callable[..., Any], *args: Any) -> Any:
    """Выполнить блокирующий вызов руки в потоке под общим замком."""
    async with app.state.arm_lock:
        return await asyncio.to_thread(func, *args)


# ---------- Модель тела запроса ----------
class PositionSet(BaseModel):
    x: int
    z: int
    angel: int
    pov_gripper: int
    gripper_state: bool

class SpeedSet(BaseModel):
    speed: int

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
    except ARMIKError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    return {"status": "Ok"}

@app.post("/api/motors/speed")
async def set_speed(request: Request, payload: SpeedSet):
    arm: RobotARM = request.app.state.arm
    try:
        await run_on_arm(request.app, arm.set_speed_arm, payload.speed)
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