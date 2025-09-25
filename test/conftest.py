import warnings
import multiprocessing

# Use 'spawn' start method to avoid fork() warnings in multi-threaded tests
try:
    multiprocessing.set_start_method('spawn')
except RuntimeError:
    # start method may already be set by another test
    pass

# Filter known harmless warnings from launch/async interactions in tests
warnings.filterwarnings(
    "ignore",
    message=r"This process .* is multi-threaded, use of fork\(\) may lead to deadlocks",
)
warnings.filterwarnings(
    "ignore",
    message=r"coroutine '.*' was never awaited",
)
warnings.filterwarnings(
    "ignore",
    message=r"There is no current event loop",
)
