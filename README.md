# Lab_04_Project
 Lab04_UART

Lab Overview: USART LED Controller
Introduction
This lab is designed to give us hands-on experience with microcontroller communications using the USART (Universal Synchronous/Asynchronous Receiver/Transmitter) interface. Our main goal is to develop a program that interacts with a user through a terminal to control LEDs on our development board.

Objectives
Learn how to handle USART communications.
Implement and understand interrupt-based data reception.
Develop a simple command parser to interpret user inputs.
Use inputs to control hardware components (LEDs).
Lab Tasks
Basic USART Communication
Receive Data: We'll start by writing code to check if data has been received via USART. This involves waiting for a status flag that tells us the receive register is not empty. It's crucial to read the received data before it gets overwritten by new incoming data.
Process Data: Upon receiving data, we'll test its value and control an LED accordingly. If the data corresponds to a specific LED color, we'll toggle that LED.
Error Handling: If a received character doesn't match any command, we'll print an error message to the console.
Code Optimization: It's important to remove any code that might interfere with our receiving process, such as delays or old transmit code, to ensure we don't miss any incoming data.
Enhancing with Interrupts
We'll upgrade our setup by enabling interrupts for data reception. This means our microcontroller can be efficient with its processing time and react immediately when data is received.
Interrupt Setup: Enable the necessary USART and NVIC configurations to handle interrupts.
Data Handling: Use an interrupt handler to save incoming data to a global variable and set a flag indicating new data is available for processing.
Advanced Command Parsing
Command Format: Implement a parser that accepts two-character commands, where the first character is an LED color and the second character is a command (0 for off, 1 for on, 2 for toggle).
User Interface: Display a prompt asking for user input and provide feedback based on the user's input, whether it's an error message or a confirmation of the action taken.
Demonstration
Basic Functionality: Show that typing a single character can control an LED as expected and that incorrect inputs generate error messages.
Interrupt-Based Reception: Demonstrate that the system can efficiently handle incoming data without losing information, even when the main loop is busy.
Command Parsing: Prove that the system correctly interprets two-character commands to control LEDs in various ways and provides appropriate feedback.
Conclusion
By completing this lab, we'll gain a deeper understanding of USART communication and how to use interrupts to create responsive and efficient embedded applications. This project also teaches us about parsing user inputs and controlling hardware based on those inputs, skills that are widely applicable in the field of embedded systems.