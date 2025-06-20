void displayMenu(const char* items[], int length, int index) {
  display.clearDisplay();
  display.setTextSize(1);
  for (int i = 0; i < length; i++) {
    if (i == index) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  // Highlight selected
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(SCREEN_MARGIN_LEFT, SCREEN_MARGIN_TOP + i * 10);
    display.print(items[i]);
    if (strcmp(items[i], "Logging") == 0) {
      if (logging) {
        display.print(": ON");
      } else {
        display.print(": OFF");
      }
    } else if (strcmp(items[i], "Top dial") == 0) {
      printDialState(dialStateTop);
    } else if (strcmp(items[i], "Bottom dial") == 0) {
      printDialState(dialStateBottom);
    } else if (strcmp(items[i], "Clear log") == 0) {
      if (clearLogPressed) {
        display.print(" (sure?)");
      }
    }
  }
  display.display();
}

void printDialState(DialState state) {
  switch (state) {
    case DialState::TEMPERATURE_IN:
      display.print(": Temp in");
      break;
    case DialState::TEMPERATURE_OUT:
      display.print(": Temp out");
      break;
    case DialState::PRESSURE:
      display.print(": Pressure");
      break;
    case DialState::HUMIDITY:
      display.print(": Humidity");
      break;
    case DialState::OFF:
      display.print(": OFF");
      break;
  }
}