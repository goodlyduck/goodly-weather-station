void displayMessage(const String &message) {

  //displayStatePrev = displayState;
  //displayState = DisplayState::MESSAGES;

  // If buffer is full, shift all lines up
  if (currentMessageLine >= DISPLAY_LINES) {
    for (int i = 1; i < DISPLAY_LINES; i++) {
      messages[i - 1] = messages[i];
    }
    currentMessageLine--;
  }

  // Add new line to buffer
  messages[currentMessageLine] = message;
  currentMessageLine++;

  // Redraw display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (int i = 0; i < currentMessageLine; i++) {
    display.setCursor(0, i * 8);  // Each line is 8 pixels tall
    display.println(messages[i]);
  }

  display.display();

  Serial.println(message);

  delay(200);
}

void displayMessageSign(String& message) {

  lastMessageSignMillis = millis();
  messageSignActive = true;

  // Calculate dimensions for the message sign window
  int border = 2;                         // Border width in pixels
  int x = 20;                         // X position of the top-left corner of the message window
  int y = 20;                              // Y position of the top-left corner of the message window
  int width = SCREEN_WIDTH - 20 * border;  // Width of the message window
  int height = 20;                        // Height of the message window

  // Draw the border and fill the message window area
  display.drawRect(x, y, width, height, SSD1306_WHITE);                                              // Border
  display.fillRect(x + border, y + border, width - 2 * border, height - 2 * border, SSD1306_BLACK);  // Inner fill

  // Set text color and size
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Calculate text position
  int textX = x + border + 4;  // Start text after the border
  int textY = y + border + 4;  // Start text after the border

  // Print the message within the bordered window
  display.setCursor(textX, textY);
  display.print(message);

  // Display the content on OLED
  display.display();
}

void displayMessageSignHours(float hours) {
  float minutes = hours * 60;
  float days = hours / 24;
  float years = days / 365;
  if (hours < 1) {
    displayMessageSign(String(minutes, 0) + " minutes");
  } else if (days > 0.99 &&  years < 1) {
    displayMessageSign(String(days, 0) + " days");
  } else if (years > 0.99) {
    displayMessageSign(String(years, 0) + " years");
  } else {
    displayMessageSign(String(hours, 0) + " hours");
  }
}