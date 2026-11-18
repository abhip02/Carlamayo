"""Shared bounded-queue inference worker for live and fixed-scene driving."""
import queue
import traceback


def run_inference_worker(requests, results, stop, process_request):
    """Process one request at a time and publish the latest result or error.

    The caller owns submission cadence, pending-request tracking, revisions,
    pose anchors, and plan expiry. Both queues should have capacity one.
    """
    while not stop.is_set():
        try:
            request = requests.get(timeout=.1)
        except queue.Empty:
            continue
        if request is None:
            break
        try:
            result = process_request(request)
        except Exception as exc:
            result = dict(frame_submitted=int(request['frame']), error=str(exc),
                          traceback=traceback.format_exc(),
                          respawn_revision=request.get('respawn_revision'))
        while True:
            try:
                results.get_nowait()
            except queue.Empty:
                break
        results.put_nowait(result)
