## `ring_buffer.hpp` Reference: Fixed-Size Circular Buffer

This header defines the `RingBuffer` template class, which implements a **fixed-size circular buffer** (also known as a circular queue). This structure is ideal for use cases requiring a history of the last $N$ items, as new data automatically overwrites the oldest data when capacity is reached.

______________________________________________________________________

### Template Class: `RingBuffer<T>`

| Type | Description |
| :--- | :--- |
| **`T`** | The type of elements to be stored within the buffer (e.g., `int`, `LogEntry`, `std::string`). |

#### Public Methods

| Method | Description |
| :--- | :--- |
| **`explicit RingBuffer(size_t capacity)`** | **Constructor.** Initializes the internal buffer storage with the specified maximum capacity. |
| **`void push(const T &item)`** | Adds a new element to the buffer (copy version). If the buffer is full, it overwrites the oldest element. |
| **`void push(T &&item)`** | Adds a new element to the buffer (move version). If the buffer is full, it overwrites the oldest element. |
| **`std::optional<T> latest() const`** | Retrieves the **most recently added element** in the buffer. Returns `std::nullopt` if the buffer is empty. |
| **`std::vector<T> getAll() const`** | Retrieves all stored elements in a new `std::vector<T>`. Elements are returned in **chronological order** (oldest to newest). |
| **`size_t size() const`** | Returns the current number of elements stored in the buffer. |
| **`bool empty() const`** | Returns `true` if no elements are currently stored (i.e., `size() == 0`). |
| **`bool full() const`** | Returns `true` if the number of stored elements equals the capacity. |

#### Private Members (Internal State)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`capacity_`** | `size_t` | The fixed maximum number of elements the buffer can hold. |
| **`buffer_`** | `std::vector<T>` | The underlying storage container for the elements. |
| **`head_`** | `size_t` | The index where the **next** element will be written (the index of the oldest element if the buffer is full). |
| **`size_`** | `size_t` | The current count of valid elements in the buffer. |
