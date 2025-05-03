#include "../include/ThreadManager.h"
#include <stdexcept>

ThreadManager::ThreadManager(size_t numThreads)
    : numThreads(numThreads), running(false)
{
    if (numThreads <= 0)
    {
        throw std::invalid_argument("Number of threads must be positive");
    }
}

ThreadManager::~ThreadManager()
{
    stop();
}

void ThreadManager::start()
{
    if (running)
        return;

    running = true;
    threads.clear();
    threadLoads.resize(numThreads);

    for (size_t i = 0; i < numThreads; ++i)
    {
        threadLoads[i] = 0;
        threads.emplace_back(&ThreadManager::workerThread, this, i);
    }
}

void ThreadManager::stop()
{
    if (!running)
        return;

    {
        std::lock_guard<std::mutex> lock(taskMutex);
        running = false;
        taskCondition.notify_all();
    }

    for (auto &thread : threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    threads.clear();
}

void ThreadManager::addTask(std::function<void()> task)
{
    if (!running)
        return;

    {
        std::lock_guard<std::mutex> lock(taskMutex);
        taskQueue.push(std::move(task));
    }

    taskCondition.notify_one();
}

bool ThreadManager::isRunning() const
{
    return running;
}

size_t ThreadManager::getNumThreads() const
{
    return numThreads;
}

void ThreadManager::setNumThreads(size_t newNumThreads)
{
    if (newNumThreads <= 0)
    {
        throw std::invalid_argument("Number of threads must be positive");
    }

    if (running)
    {
        stop();
        numThreads = newNumThreads;
        start();
    }
    else
    {
        numThreads = newNumThreads;
    }
}

size_t ThreadManager::getTaskCount() const
{
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion()
{
    std::unique_lock<std::mutex> lock(completionMutex);
    while (getTaskCount() > 0 || activeThreads > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

size_t ThreadManager::getActiveThreadCount() const
{
    return activeThreads;
}

void ThreadManager::processNextTask()
{
    std::function<void()> task;

    {
        std::unique_lock<std::mutex> lock(taskMutex);
        if (taskQueue.empty())
            return;

        task = std::move(taskQueue.front());
        taskQueue.pop();
    }

    if (task)
    {
        activeThreads++;
        task();
        activeThreads--;
    }
}

void ThreadManager::workerThread(size_t threadId)
{
    while (running)
    {
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock(taskMutex);
            taskCondition.wait(lock, [this]
                               { return !taskQueue.empty() || !running; });

            if (!running && taskQueue.empty())
            {
                return;
            }

            if (!taskQueue.empty())
            {
                task = std::move(taskQueue.front());
                taskQueue.pop();
                threadLoads[threadId]++;
            }
        }

        if (task)
        {
            activeThreads++;
            task();
            activeThreads--;
        }
    }
}