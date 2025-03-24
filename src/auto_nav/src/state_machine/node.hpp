#pragma once

namespace FSM
{
    class Node
    {
    public:
        virtual ~Node() = default;
        virtual void onEnter() = 0;
        virtual void onExit() = 0;
    };
}