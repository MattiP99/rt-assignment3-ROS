#include <iostream>
#include <string>
#include <sstream>
#include <termios.h>
#include <future>
#include <thread>
#include <chrono>
#include <unistd.h>


/// Some utility functions



void clearInputBuffer() {
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
}
/**
 * Changes color of terminal text
 * @param colorCode  ANSI color code
 * @param isBold     Bold text or not
 */
void terminalColor(const char colorCode) {
  

  //std::cout << "\033[" << specialCode << colorCode << "m";
  //fflush(stdout);
  std::string result;
  //std::string s;
  //char ch = colorCode;
  //s = "color ";
  //s_final = s.push_back(ch);
  std::stringstream stream;
  stream << "color "<<colorCode<<"B";
 result = stream.str();// std::cout << "Command to execute: '" << stream.str() << "'\n";
  system(result.c_str());
}

/**
 * Clears the terminal and resets it to initial state
 */
void clearTerminal() {
  //printf("\033c");
  system("clear");
  terminalColor('4');
  printf("==========================\n");
  printf("= MOBILE ROBOT CONTROLLER =\n");
  printf("==========================\n\n");
  terminalColor('4');
  fflush(stdout);
}

/**
 * Prints text with typing effect
 * @param str    Text to be printed
 * @param delay  Delay between each character, in microseconds
 */
void displayText (std::string str, int delay) {
  for (int i = 0; i < str.length(); i++) {
    std::cout << str[i];
    fflush(stdout);
    usleep(delay);
  }
}

/**
 * Check if string is numeric or not
 */
bool isStringNumeric(std::string str) {
  for (int i = 0; i < str.length(); i++) {
    if (std::isdigit(str[i])) {
      return true;
    }
  }

  return false;
}
