#include <iostream>
#include <random>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
    std::unique_lock<std::mutex> uniqueLock(_mutex);
    // Wait until there's a message in the queue.
    _condition.wait(uniqueLock, [this] {return !_queue.empty(); });
    // Take out the message and return it.
    T message = std::move(_queue.back());
    _queue.pop_back();
    return message;
}

template <typename T>
void MessageQueue<T>::send(T&& message)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    std::lock_guard<std::mutex> lockGuard(_mutex);
    // Add message to queue.
    _queue.push_back(std::move(message));
    // Notify client after pushing new message into queue.
    _condition.notify_one();
}


/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight(): _currentPhase{TrafficLightPhase::red} {}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    // synchronize concurrent access to _currentPhase
    std::lock_guard<std::mutex> lockGuard(_currentPhaseMutex);
    return _currentPhase;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    while (_phaseQueue.receive() == TrafficLightPhase::red);
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method âcycleThroughPhasesâ should be started in a thread when the public method âsimulateâ is called. To do this, use the thread queue in the base class. 

    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles.

    int cycleDuration = getRandomDuration(); // duration of a single simulation cycle in ms

    // init stop watch
    auto lastUpdate = std::chrono::system_clock::now();

    while (true) {
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();

        if (timeSinceLastUpdate >= cycleDuration)
        {
            // synchronize concurrent access to _currentPhase
            std::lock_guard<std::mutex> lockGuard(_currentPhaseMutex);
            // Toggle current phase.
            _currentPhase = _currentPhase == TrafficLightPhase::green ? TrafficLightPhase::red : TrafficLightPhase::green;
            _phaseQueue.send(std::move(_currentPhase));
            // reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();
            // pick a new random duration
            cycleDuration = getRandomDuration();
        }
    }
}

// Returns a random duration between 4000 to 6000 milliseconds (4 to 6 seconds).
int TrafficLight::getRandomDuration()
{
    constexpr int minMilliseconds = 4000; // 4 seconds;
    constexpr int maxMilliseconds = 6000; // 6 seconds;
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(minMilliseconds, maxMilliseconds);
    return distr(eng);
}
