## `CameraModule.h` Reference: Threaded Camera Capture Module

This header defines the `CameraModule` class, which manages capturing image frames in a dedicated background thread, providing thread-safe access to the latest frames and optional binary logging.

______________________________________________________________________

### Class: `CameraModule`

| Feature | Description |
| :--- | :--- |
| **Capture Source** | Uses the `lccv::PiCamera` class to interface with the camera device. |
| **Threaded Operation** | Runs a background thread (`captureLoop`) to continuously grab frames. |
| **Data Buffer** | Stores recent frames in a `RingBuffer<TimedFrame>` for history and asynchronous access. |
| **Thread Safety** | Uses a `std::mutex` for safe buffer access and a `std::condition_variable` for blocking waits. |

#### Public Type

| Type | Description |
| :--- | :--- |
| **`CameraOptionCallback`** | `std::function<void(lccv::PiCamera &)>`. A functional type used to pass custom configuration settings to the internal `lccv::PiCamera` instance during initialization or runtime. |

#### Public Methods

| Method | Description |
| :--- | :--- |
| **`CameraModule(CameraOptionCallback callback)`** | **Constructor (No Logging).** Initializes the module and configures the camera. Does not start the capture thread. |
| **`CameraModule(Logger *logger, CameraOptionCallback callback)`** | **Constructor (With Logging).** Initializes the module, configures the camera, and stores the provided pointer to an external `Logger` instance. |
| **`~CameraModule()`** | **Destructor.** Safely calls `stop()` to ensure the capture thread is stopped and joined. |
| **`void changeSetting(CameraOptionCallback callback)`** | Applies a new configuration to the internal camera instance, allowing dynamic changes (e.g., resolution, exposure). |
| **`bool start()`** | Starts the dedicated background thread that executes the frame capture loop. Returns `true` on success. |
| **`void stop()`** | Stops the capture loop, signals the thread to exit, and blocks until the thread has finished execution (`join`). |
| **`bool getFrame(TimedFrame &outTimedFrame) const`** | **Non-blocking read.** Retrieves the most recently captured `TimedFrame` (image and timestamp) from the internal buffer. Returns `true` if a frame is available. |
| **`size_t bufferSize() const`** | Returns the current number of frames stored in the internal `RingBuffer`. |
| **`bool getAllTimedFrame(std::vector<TimedFrame> &outTimedFrames) const`** | Retrieves **all** frames currently in the buffer, ordered from oldest to newest. Returns `true` if the buffer is non-empty. |
| **`bool waitForFrame(TimedFrame &outTimedFrame)`** | **Blocking read.** Suspends the calling thread until a new frame is captured, utilizing the condition variable to notify of new data. |
| **`void startLogging()`** | Enables binary logging of all subsequently captured frames to the configured `Logger` instance. |
| **`void stopLogging()`** | Disables binary logging of captured frames. |

#### Private Members (Internal State)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`captureLoop()`** | `void` | Background thread function responsible for continuous capture, buffering, signaling, and optional logging. |
| **`cam_`** | `lccv::PiCamera` | The underlying camera interface instance. |
| **`cameraThread_`** | `std::thread` | The background thread running the capture loop. |
| **`running_`** | `std::atomic<bool>` | Atomic flag controlling the execution state of the capture loop. |
| **`frameMutex_`** | `std::mutex` | Mutex protecting access to the `frameBuffer_` and condition variable. |
| **`frameUpdated_`** | `std::condition_variable` | Used to notify waiting threads (e.g., in `waitForFrame`) whenever a new frame is captured. |
| **`frameBuffer_`** | `RingBuffer<TimedFrame>` | Circular buffer that stores the last $30$ captured frames and their timestamps. |
| **`logger_`** | `Logger*` | Pointer to the system logger instance. |
| **`logging_`** | `bool` | Flag indicating if frame data is currently being logged. |
