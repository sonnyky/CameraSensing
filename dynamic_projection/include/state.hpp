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
		current_state = &std::get<state_type>(possible_states);
	}

	template <typename Event>
	void handle(const Event& event)
	{
		auto passEventToState = [this, &event](auto statePtr) {
			statePtr->state_handler(event).execute(*this);
		};
		std::visit(passEventToState, current_state);
	}

	template<typename... states>
	auto get_current_state()
	{
		return current_state;
	}

private:
	std::tuple<states...> possible_states;
	std::variant<states*...> current_state{ &std::get<0>(possible_states) };
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
struct CalibrationState;

struct IdleState
{
	TransitionTo<TrackingState> state_handler(const IdleToTrackingEvent&) const
	{
		std::cout << "Transitioning from IdleState to TrackingState..." << std::endl;
		return {};
	}

	TransitionTo<CalibrationState> state_handler(const IdleToCalibrationEvent&) const
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

struct CalibrationState
{
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

using CaptureStateManager = state_machine<IdleState, TrackingState, CalibrationState>;

