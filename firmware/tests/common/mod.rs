pub mod mock_endpoint;
pub mod test_harness;

use std::future::Future;
use std::pin::pin;
use std::sync::{Arc, Mutex, MutexGuard};
use std::task::{Context, Poll, Wake, Waker};

use embassy_time::{Duration, MockDriver};

static TEST_LOCK: Mutex<()> = Mutex::new(());

pub fn test_lock() -> MutexGuard<'static, ()> {
    TEST_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

struct NoopWake;

impl Wake for NoopWake {
    fn wake(self: Arc<Self>) {}
}

pub fn run_async<F: Future>(future: F) -> F::Output {
    MockDriver::get().reset();

    let waker = Waker::from(Arc::new(NoopWake));
    let mut future = pin!(future);

    for _ in 0..200_000 {
        match future.as_mut().poll(&mut Context::from_waker(&waker)) {
            Poll::Ready(output) => return output,
            Poll::Pending => MockDriver::get().advance(Duration::from_millis(1)),
        }
    }

    panic!("async test did not complete within poll budget")
}
