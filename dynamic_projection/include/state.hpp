#include <iostream>
#include <vector>

#include <tuple>
#include <boost/any.hpp>

template <typename... states>
class state_machine {
public :

	template<typename state_type>
	void transition_to()
	{
		current_state = &std::get<state_type>(states);
	}

	// Update is executed every frame
	template<typename update_type>
	update_type update();

	template <typename event>
	void handle(const event& event)
	{
		auto passEventToState = [&event](auto statePtr) {
			statePtr->state_handler(event);
		};
		std::visit(passEventToState, currentState);
	}
	
	template <typename state_type>
	struct TransitionTo
	{
		template <typename Machine>
		void execute(Machine& machine)
		{
			machine.template transition_to<state_type>();
		}
	};

private:
	std::tuple<states...> states;
	boost::any current_state{ &std::get<0>(states)};

};

template <typename State>
struct TransitionTo
{
	template <typename Machine>
	void execute(Machine& machine)
	{
		machine.template transitionTo<State>();
	}
};

struct Nothing
{
	template <typename Machine>
	void execute(Machine&)
	{
	}
};

struct OpenEvent
{
};

struct CloseEvent
{
};

struct ClosedState;
struct OpenState;

struct ClosedState
{
	TransitionTo<OpenState> state_handler(const OpenEvent&) const
	{
		std::cout << "Opening the door..." << std::endl;
		return {};
	}

	Nothing state_handler(const CloseEvent&) const
	{
		std::cout << "Cannot close. The door is already closed!" << std::endl;
		return {};
	}
};

struct OpenState
{
	Nothing state_handler(const OpenEvent&) const
	{
		std::cout << "Cannot open. The door is already open!" << std::endl;
		return {};
	}

	TransitionTo<ClosedState> state_handler(const CloseEvent&) const
	{
		std::cout << "Closing the door..." << std::endl;
		return {};
	}
};
