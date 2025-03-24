#include "machine.hpp"

void FSM::Machine::addNode(std::unique_ptr<Node> node)
{
    nodes.push_back(std::move(node));
}

void FSM::Machine::addTransition(std::unique_ptr<Transition> transition)
{
    transitions.push_back(std::move(transition));
}

void FSM::Machine::start()
{
    if (nodes.empty())
    {
        throw std::runtime_error("No nodes added to machine");
    }

    if (currentNode != nullptr)
    {
        throw std::runtime_error("Machine already started");
    }

    currentNode = nodes.front().get();
    currentNode->onEnter();
}

void FSM::Machine::instruct(Instruction instruction)
{
    if (currentNode == nullptr)
    {
        throw std::runtime_error("Machine not started");
    }

    for (const auto &transition : transitions)
    {
        if (transition->isTriggered(currentNode, instruction))
        {
            currentNode->onExit();
            currentNode = transition->getTo();
            currentNode->onEnter();

            return;
        }
    }
}
