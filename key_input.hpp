#pragma once

#include <thread>

class KeyInput{
public:
    KeyInput();
    ~KeyInput();
    void Start();
    char GetKey();
private:
    std::thread m_th;
    char m_c;
    void SetTerm();
};
