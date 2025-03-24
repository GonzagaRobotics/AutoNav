#pragma once

#include <stdexcept>
#include "auto_nav_interfaces/Types.hpp"
#include "node.hpp"

namespace FSM
{
    class Transition
    {
    private:
        Node *from;
        Node *to;
        Instruction instruction;

    public:
        Transition() = delete;

        Transition(Node *from, Node *to, Instruction instruction)
        {
            if (from == nullptr || to == nullptr)
            {
                throw std::invalid_argument("Nodes cannot be null");
            }

            this->from = from;
            this->to = to;
            this->instruction = instruction;
        }

        bool isTriggered(Node *from, Instruction instruction)
        {
            return this->from == from && this->instruction == instruction;
        }

        Node *getTo()
        {
            return to;
        }
    };
}