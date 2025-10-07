# Robot FSM Architecture Documentation

## Overview

This FSM manages robot modes or operational states using a type-safe, variant-based approach.\
The architecture separates data into **three lifetimes** to make behavior predictable and maintainable:

- **Context**: ephemeral, recomputed every tick
- **StateContext**: persistent across states
- **State-local data**: temporary per-state

### FSM Flow Diagram

```
Tick Start
   │
   ▼
Update Context (compute ephemeral data)
   │
   ▼
FSM update() -> handle(current state)
   │
   ├─> may transition state
   │
   ▼
Tick End
```

## Data Lifetimes

| Struct | Lifetime | Read/Write | Purpose | Examples |
|--------------|----------------|----------------|--------------------------------|------------------------------------------|
| `Context` | per-tick | read-only | ephemeral input for FSM | sensor readings, computed deltas, temporary measurements |
| `StateContext` | persistent | read/write | shared memory across states | robot mode, counters, configuration data |
| State-local | per-state | read/write | temporary state info | timers, session counters, intermediate calculations |

### Context (per-tick)

- Recomputed each update
- Passed by `const &` to state handlers
- Read-only inside states
- Examples: sensor readings, filtered inputs, computed deltas

### StateContext (persistent)

- Lives across states
- Writable by state handlers
- Examples: current mode, number of iterations, last known heading

### State-local data (temporary)

- Stored inside individual states
- Reset when entering a new state
- Examples: session timers, burst counters, temporary flags

## FSM Mechanics

- Each state is a struct stored in a `std::variant` or similar type-safe container
- State handlers are invoked each tick via a visitor or dispatch mechanism
- `Context` is passed by `const &`
- `StateContext` is passed by reference
- Instant transitions can be optionally enabled

### State Handler Signature

```cpp
void handle(StateType &state, const Context &ctx, StateContext &stateCtx, bool &transitioned);
```

- `state`: current state struct
- `ctx`: read-only per-tick data
- `stateCtx`: persistent FSM-wide data
- `transitioned`: set to `true` if state changes instantly

## State Transition Diagram

```
StateA ---[conditionA]---> StateB ---[conditionB]---> StateC ---[conditionC]---> StateA
```

> This is an example; actual transitions depend on your robot's logic.

## Best Practices

1. Never store persistent variables in `Context` — it resets every tick
1. Keep `Context` immutable by passing it `const &`
1. Use `StateContext` for any variable that should persist across states
1. Use state-local variables for timers, counters, and session-specific info
1. Name variables to make lifetimes obvious:
   - `state.*` → temporary per-state
   - `ctx.*` → ephemeral input
   - `stateCfx.*` → persistent
1. For complex handlers, consider a helper struct `HandleArgs` to group parameters

## Example

```cpp
#include <chrono>
#include <iostream>
#include <thread>
#include <typeindex>
#include <variant>

using Clock = std::chrono::steady_clock;

// --- Contexts --- //
struct Context {
    // ephemeral per-tick variables (read-only inside states)
    Clock::time_point now;
    // Add more per-tick computed variables here
};

struct StateContext {
    // persistent FSM-wide variables (shared across states)
    int allStatesTimeEntered = 0;
    // Add shared persistent variables here
};

// Function to update per-tick context
void updateContext(Context &ctx) {
    ctx.now = Clock::now();
    // compute other ephemeral per-tick variables here
}

// --- States --- //
struct Idle {
    static int timesEntered;      // persistent
    Clock::time_point enterTime;  // temporary
};
int Idle::timesEntered = 0;

struct Walking {
    static float totalDistance;   // persistent
    float sessionDistance = 0.f;  // temporary
};
float Walking::totalDistance = 0.f;

struct Running {
    static int totalBursts;  // persistent
    int burst = 0;           // temporary
};
int Running::totalBursts = 0;

using State = std::variant<Idle, Walking, Running>;

void idleHandle(Idle &state, bool isNew, bool &instantTransitioned, const Context &ctx, StateContext &stateCtx, State &currentState) {
    if (isNew) {
        std::cout << "[Idle] Started\n";
        state.enterTime = ctx.now;
        state.timesEntered++;
        stateCtx.allStatesTimeEntered++;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(ctx.now - state.enterTime).count();

    std::cout << "[Idle] elapsed = " << elapsed << " | timesEntered = " << state.timesEntered
              << " | allTimeEntered = " << stateCtx.allStatesTimeEntered << "\n";

    if (elapsed >= 3) {
        currentState = Walking{};
        instantTransitioned = true;
    }
}

void walkingHandle(Walking &state, bool isNew, bool &instantTransitioned, const Context &ctx, StateContext &stateCtx, State &currentState) {
    if (isNew) {
        std::cout << "[Walking] Started\n";
        stateCtx.allStatesTimeEntered++;
    }

    state.sessionDistance += 1.0f;
    state.totalDistance += 1.0f;

    std::cout << "[Walking] sessionDistance = " << state.sessionDistance << " | totalDistance = " << state.totalDistance
              << " | allTimeEntered = " << stateCtx.allStatesTimeEntered << "\n";

    if (state.sessionDistance >= 3.0f) {
        currentState = Running{};
        instantTransitioned = true;
    }
}

void runningHandle(Running &state, bool isNew, bool &instantTransitioned, const Context &ctx, StateContext &stateCtx, State &currentState) {
    if (isNew) {
        std::cout << "[Running] Started\n";
        stateCtx.allStatesTimeEntered++;
    }

    state.burst++;
    state.totalBursts++;

    std::cout << "[Running] burst = " << state.burst << " | totalBursts = " << state.totalBursts
              << " | allTimeEntered = " << stateCtx.allStatesTimeEntered << "\n";

    if (state.burst >= 2) {
        currentState = Idle{};
        instantTransitioned = true;
    }
}

// --- FSM --- //
struct FSM {
    State currentState{Idle{}};
    std::type_index lastStateType = typeid(void);
    StateContext stateCtx;  // persistent FSM-wide data

    void update(const Context &ctx) {
        bool instantTransitioned;
        int safetyCounter = 0;

        do {
            instantTransitioned = false;

            std::type_index currentType = std::visit([](auto &s) -> std::type_index { return typeid(s); }, currentState);
            bool isNewState = (currentType != lastStateType);
            lastStateType = currentType;

            std::visit(
                [this, isNewState, &instantTransitioned, &ctx](auto &s) { handle(s, isNewState, instantTransitioned, ctx, stateCtx); },
                currentState
            );

            if (++safetyCounter > 10) {
                std::cerr << "Warning: too many instant transitions in one update!\n";
                break;
            }

        } while (instantTransitioned);
    }

    void handle(Idle &state, bool isNew, bool &instantTransitioned, const Context &ctx, StateContext &stateCtx) {
        idleHandle(state, isNew, instantTransitioned, ctx, stateCtx, currentState);
    }

    void handle(Walking &state, bool isNew, bool &instantTransitioned, const Context &ctx, StateContext &stateCtx) {
        walkingHandle(state, isNew, instantTransitioned, ctx, stateCtx, currentState);
    }

    void handle(Running &state, bool isNew, bool &instantTransitioned, const Context &ctx, StateContext &stateCtx) {
        runningHandle(state, isNew, instantTransitioned, ctx, stateCtx, currentState);
    }
};

// --- Main --- //
int main() {
    FSM fsm;
    Context ctx;

    for (int i = 0; i < 20; ++i) {
        std::cout << "=== Tick " << i << " ===\n";

        updateContext(ctx);  // recompute per-tick ephemeral data
        fsm.update(ctx);     // update FSM using latest context

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
```

This pattern ensures proper separation of data lifetimes and makes the FSM robust, maintainable, and readable.
