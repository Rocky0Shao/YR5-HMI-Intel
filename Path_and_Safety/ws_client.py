# ws_client.py
import asyncio
import threading
from typing import Optional, Callable

class WSClient:
    def __init__(self, uri: str, on_error: Optional[Callable[[str], None]] = None):
        self.uri = uri
        self.on_error = on_error or (lambda msg: None)
        self.loop = asyncio.new_event_loop()
        self.queue: "asyncio.Queue[bytes]" = asyncio.Queue()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self._stopping = threading.Event()

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stopping.set()
        def _stop():
            for t in asyncio.all_tasks(self.loop):
                t.cancel()
            self.loop.stop()
        self.loop.call_soon_threadsafe(_stop)
        self.thread.join(timeout=2)

    def send(self, data: bytes) -> None:
        # thread-safe enqueue from ROS callbacks/timers
        asyncio.run_coroutine_threadsafe(self.queue.put(data), self.loop)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._main())
        finally:
            self.loop.close()

    async def _main(self) -> None:
        import websockets  # lazy import to avoid blocking ROS import path
        while not self._stopping.is_set():
            try:
                async with websockets.connect(
                    self.uri,
                    max_size=None,
                    ping_interval=20,
                    ping_timeout=20,
                ) as ws:
                    # Drain queue and send each payload as a binary frame
                    while not self._stopping.is_set():
                        data = await self.queue.get()
                        await ws.send(data)  # binary by default for bytes
            except Exception as e:
                self.on_error(f"WS reconnect in 1s: {e!r}")
                await asyncio.sleep(1.0)
