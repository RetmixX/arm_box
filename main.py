import asyncio
import os
from contextlib import asynccontextmanager, suppress
from typing import Any, Callable, Literal, Optional

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, ValidationError

from arm_function import ARMIKError, RobotARM

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
    arm = await asyncio.to_thread(RobotARM, device=device_path, speed=10)
    app.state.arm = arm
    app.state.arm_lock = asyncio.Lock()
    yield
    # Остановка сервиса: закрываем порт
    await asyncio.to_thread(arm.close)


# host/port задаются при запуске uvicorn (--host 0.0.0.0 --port 8000), не здесь
app = FastAPI(title="Motors API", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


async def run_on_arm(app: FastAPI, func: Callable[..., Any], *args: Any) -> Any:
    async with app.state.arm_lock:
        return await asyncio.to_thread(func, *args)


# ---------- Модель тела запроса ----------
class PositionSet(BaseModel):
    x: int
    z: int
    angel: int
    pov_gripper: int
    gripper_state: bool


class PositionSetPaint(BaseModel):
    x: int
    z: int
    angel: int
    pov_gripper: int
    paint_pov: int


class SpeedSet(BaseModel):
    speed: int = Field(ge=1, le=100, description="Скорость, %")


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


# ---------- POST-маршруты ----------
@app.post("/api/motors/pos/set")
async def set_pos(request: Request, payload: PositionSet):
    arm: RobotARM = request.app.state.arm
    try:
        await run_on_arm(request.app, arm.move_arm, payload.x, payload.z, payload.angel,
                         payload.pov_gripper, payload.gripper_state)
    except ARMIKError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    return {"status": "Ok"}


@app.post("/api/motors/pos/set-paint")
async def set_pos_paint(request: Request, payload: PositionSetPaint):
    arm: RobotARM = request.app.state.arm
    try:
        await run_on_arm(request.app, arm.move_arm_paint, payload.x, payload.z, payload.angel,
                         payload.pov_gripper, payload.paint_pov)
    except ARMIKError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    return {"status": "Ok"}


@app.post("/api/motors/speed")
async def set_speed(request: Request, payload: SpeedSet):
    arm: RobotARM = request.app.state.arm
    try:
        await run_on_arm(request.app, arm.set_speed, payload.speed)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    return {"status": "Ok"}


# ---------- WebSocket ----------
#
# Движение длится, пока рука не придёт в цель. Если клиент присылает команды
# быстрее, чем рука их выполняет, старые команды НЕ копятся в очереди:
# выполняется текущая, а из пришедших за это время берётся только последняя.
# На каждую команду клиент получает ровно один ответ: ok, error или skipped.

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    print("client connected: ", ws.client)
    arm: RobotARM = ws.app.state.arm

    latest: Optional[WsCommand] = None
    has_cmd = asyncio.Event()
    send_lock = asyncio.Lock()

    async def send(data: dict):
        async with send_lock:
            await ws.send_json(data)

    async def worker():
        nonlocal latest
        while True:
            await has_cmd.wait()
            has_cmd.clear()
            cmd, latest = latest, None
            if cmd is None:
                continue
            try:
                await run_on_arm(ws.app, arm.move_arm, cmd.x, cmd.z, cmd.angel,
                                 cmd.pov_gripper, cmd.gripper_state)
            except Exception as e:
                print("move failed: ", e)
                await send({"status": "error", "message": str(e)})
                continue
            await send({"status": "ok", "message": "move completed"})

    worker_task = asyncio.create_task(worker())
    try:
        while True:
            raw = await ws.receive_text()
            try:
                cmd = WsCommand.model_validate_json(raw)   # и JSON, и поля
            except ValidationError as e:
                await send({"status": "error", "message": f"bad request: {e.errors()}"})
                continue

            if latest is not None:
                # Предыдущая ещё не начата - заменяем её новой
                await send({"status": "skipped", "message": "replaced by newer command"})
            latest = cmd
            has_cmd.set()

    except WebSocketDisconnect:
        print("client disconnected: ", ws.client)
    finally:
        worker_task.cancel()
        with suppress(asyncio.CancelledError):
            await worker_task