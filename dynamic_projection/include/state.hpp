#include <iostream>
#include <vector>
#include <variant>
#include <optional>
#include <tuple>

template <typename... states>
class state_machine {
public :

	template<typename state_type>
	void transition_to()
	{
		// Call on_exit() if it exists
		std::visit([](auto* state_ptr) {
			using T = std::decay_t<decltype(*state_ptr)>;
			if (has_on_exit<T>::value) {
				state_ptr->on_exit();
			}
			}, current_state);

		// Switch state
		current_state = &std::get<state_type>(possible_states);

		// Call on_enter() if it exists
		std::visit([](auto* state_ptr) {
			using T = std::decay_t<decltype(*state_ptr)>;
			if (has_on_enter<T>::value) {
				state_ptr->on_enter();
			}
			}, current_state);
	}

	template <typename Event>
	void handle(const Event& event)
	{
		auto passEventToState = [this, &event](auto statePtr) {
			statePtr->state_handler(event).execute(*this);
		};
		std::visit(passEventToState, current_state);
	}

	auto get_current_state()
	{
		return current_state;
	}

	std::string current_state_name() const
	{
		return std::visit([](auto statePtr) {
			using T = std::decay_t<decltype(*statePtr)>;
			if constexpr (std::is_same_v<T, IdleState>) return "IdleState";
			else if constexpr (std::is_same_v<T, TrackingState>) return "TrackingState";
			else if constexpr (std::is_same_v<T, CameraCalibrationState>) return "CalibrationState";
			else return "UnknownState";
			}, current_state);
	}

private:
	std::tuple<states...> possible_states;
	std::variant<states*...> current_state{ &std::get<0>(possible_states) };
};

template<typename T>
class has_on_enter {
private:
	template<typename U>
	static auto test(int) -> decltype(std::declval<U>().on_enter(), std::true_type{});

	template<typename>
	static std::false_type test(...);

public:
	static constexpr bool value = decltype(test<T>(0))::value;
};

// Check if T has a method called on_exit()
template<typename T>
class has_on_exit {
private:
	template<typename U>
	static auto test(int) -> decltype(std::declval<U>().on_exit(), std::true_type{});

	template<typename>
	static std::false_type test(...);

public:
	static constexpr bool value = decltype(test<T>(0))::value;
};

template <typename state_type>
struct TransitionTo
{
	template <typename Machine>
	void execute(Machine& machine)
	{
		machine.template transition_to<state_type>();
	}
};

struct Nothing
{
	template <typename Machine>
	void execute(Machine&)
	{
	}
};

struct IdleToCalibrationEvent
{
};

struct IdleToTrackingEvent
{
};

struct CalibrationToIdleEvent
{};

struct TrackingToIdleEvent{};


struct TrackingState;
struct CameraCalibrationState;

struct IdleState
{
	void on_enter() const { std::cout << "Entering IdleState\n"; }
	void on_exit() const { std::cout << "Exiting IdleState\n"; }

	TransitionTo<TrackingState> state_handler(const IdleToTrackingEvent&) const
	{
		std::cout << "Transitioning from IdleState to TrackingState..." << std::endl;
		return {};
	}

	TransitionTo<CameraCalibrationState> state_handler(const IdleToCalibrationEvent&) const
	{
		std::cout << "Transitioning from IdleState to CalibrationState..." << std::endl;
		return {};
	}

	Nothing state_handler(...) const
	{
		std::cout << "IdleState: Event not recognized." << std::endl;
		return {};
	}
};

struct TrackingState
{
	void on_enter() const { std::cout << "Entering TrackingState\n"; }
	void on_exit() const { std::cout << "Exiting TrackingState\n"; }
	TransitionTo<IdleState> state_handler(const TrackingToIdleEvent&) const
	{
		std::cout << "Transitioning from TrackingState to IdleState..." << std::endl;
		return {};
	}

	Nothing state_handler(const IdleToTrackingEvent&) const
	{
		std::cout << "Cannot close. The door is already closed!" << std::endl;
		return {};
	}
};

struct CameraCalibrationState
{
	void on_enter() const { std::cout << "Entering CalibrationState\n"; }
	void on_exit() const { std::cout << "Exiting CalibrationState\n"; }
	Nothing state_handler(const IdleToCalibrationEvent&) const
	{
		std::cout << "Cannot open. The door is already open!" << std::endl;
		return {};
	}

	TransitionTo<IdleState> state_handler(const CalibrationToIdleEvent&) const
	{
		std::cout << "Transitioning from CalibrationState to IdleState..." << std::endl;
		return {};
	}
};

using CaptureStateManager = state_machine<IdleState, TrackingState, CameraCalibrationState>;

