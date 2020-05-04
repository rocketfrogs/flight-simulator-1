#pragma once

// Initialize the encoder
void EncoderBegin(int pinA, int pinB);

// Return the relative movement in number of encoder steps since the last call to EncoderGetMovement()
int EncoderGetMovement();
