# ws_client.py
import asyncio, threading, random, time
from typing import Optional, Callable

class WSClient:
    def __init__(self, uri: str, on_error: Optional[Callable[[str], None]] = None,
                 queue_max: int = 256, drop_oldest: bool = True):
        self.uri = uri
        self.on_error = on_error or (lambda msg: None)
        self.loop = asyncio.new_event_loop()
        self.queue: "asyncio.Queue[bytes]" = asyncio.Queue(maxsize=queue_max)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self._stopping = threading.Event()
        self._started = threading.Event()
        self.drop_oldest = drop_oldest

    def start(self) -> None:
        self.thread.start()
        self._started.wait(timeout=2)  # loop is ready

    def stop(self) -> None:
        self._stopping.set()
        # wake any pending queue.get()
        try:
            self.send(b"")  # harmless no-op frame; will be ignored if you filter zeros
        except Exception:
            pass
        def _stop():
            for t in list(asyncio.all_tasks(self.loop)):
                t.cancel()
            self.loop.stop()
        self.loop.call_soon_threadsafe(_stop)
        self.thread.join(timeout=2)

    def send(self, data: bytes) -> None:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("WSClient.send expects bytes-like payload")
        if not self._started.is_set():
            # Optional: buffer locally until loop is ready
            time.sleep(0.01)
        # Thread-safe enqueue with backpressure policy
        async def _put(q: asyncio.Queue, payload: bytes):
            if q.full():
                if self.drop_oldest:
                    try:
                        q.get_nowait()
                        q.task_done()
                    except asyncio.QueueEmpty:
                        pass
                else:
                    await q.put(payload); return
            try:
                q.put_nowait(bytes(payload))
            except asyncio.QueueFull:
                # fall back: drop this frame
                pass
        asyncio.run_coroutine_threadsafe(_put(self.queue, data), self.loop)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        self._started.set()
        try:
            self.loop.run_until_complete(self._main())
        finally:
            self.loop.close()

    async def _main(self) -> None:
        import websockets
        backoff = 1.0
        while not self._stopping.is_set():
            try:
                async with websockets.connect(
                    self.uri,
                    max_size=None,
                    ping_interval=20,
                    ping_timeout=20,
                ) as ws:
                    backoff = 1.0  # reset on success
                    while not self._stopping.is_set():
                        data = await self.queue.get()
                        try:
                            if not data:
                                # optional: ignore empty wake-up signals
                                self.queue.task_done()
                                continue
                            await ws.send(data)
                        finally:
                            self.queue.task_done()
            except (asyncio.CancelledError, KeyboardInterrupt):
                break
            except Exception as e:
                self.on_error(f"WS error: {type(e).__name__}: {e!s}")
                # Exponential backoff with jitter, capped
                await asyncio.sleep(backoff + random.random() * 0.5)
                backoff = min(backoff * 2, 10.0)
