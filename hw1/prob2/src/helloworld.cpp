#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "prime-checker.hpp"
#include "messages.hpp"

int32_t main(int32_t, char **) {
    PrimeChecker pc;
    std::cout << "Hello world! = " << pc.isPrime(43) << std::endl;

    cluon::OD4Session od4(111,
                          [](cluon::data::Envelope &&envelope) {
                              if (envelope.dataType() == 2001) {
                                  MyTestMessage1 receivedMsg = cluon::extractMessage<MyTestMessage1>(
                                          std::move(envelope));
                                  PrimeChecker pc;
                                  std::cout << receivedMsg.myValue() << " is "
                                            << (pc.isPrime(receivedMsg.myValue()) ? "" : "not") << " a prime. "
                                            << std::endl;
                              }
                          });

    cluon::UDPSender sender{"225.0.0.111", 1238};

    uint16_t value;
    std::cout << "Enter a number to check: ";
    std::cin >> value;
    MyTestMessage1 msg;
    msg.myValue(value);
    od4.send(msg);
    // Sleep so that we have time to receive our own message
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
    return 0;
}
