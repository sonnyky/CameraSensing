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

struct StartCalibrationEvent
{
};

struct StartTrackingEvent
{
};

struct TrackingState;
struct CalibrationState;

struct TrackingState
{
	TransitionTo<CalibrationState> state_handler(const StartCalibrationEvent&) const
	{
		std::cout << "Opening the door..." << std::endl;
		return {};
	}

	Nothing state_handler(const StartTrackingEvent&) const
	{
		std::cout << "Cannot close. The door is already closed!" << std::endl;
		return {};
	}
};

struct CalibrationState
{
	Nothing state_handler(const StartCalibrationEvent&) const
	{
		std::cout << "Cannot open. The door is already open!" << std::endl;
		return {};
	}

	TransitionTo<TrackingState> state_handler(const StartTrackingEvent&) const
	{
		std::cout << "Closing the door..." << std::endl;
		return {};
	}
};

using CaptureStateManager = state_machine<TrackingState, CalibrationState>;

