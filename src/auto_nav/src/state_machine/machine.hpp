#pragma once

#include <memory>
#include <vector>
#include "auto_nav_interfaces/Types.hpp"
#include "node.hpp"
#include "transition.hpp"

namespace FSM
{
    class Machine
    {
    private:
        std::vector<std::unique_ptr<Node>> nodes;
        std::vector<std::unique_ptr<Transition>> transitions;

        Node *currentNode = nullptr;

    public:
        void addNode(std::unique_ptr<Node> node);
        void addTransition(std::unique_ptr<Transition> transition);

        void start();
        void instruct(Instruction instruction);
    };
}
