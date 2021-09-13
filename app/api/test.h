#include <iostream>

class test
{
public:
    test();
    test(test &&) = default;
    test(const test &) = default;
    test &operator=(test &&) = default;
    test &operator=(const test &) = default;
    void testing();
    ~test();

private:

};

void test::testing(){
    std::cout << "Shit works!";
}

test::test()
{
}

test::~test()
{
}
