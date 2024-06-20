void displayMenu(const char* items[], int length, int index) {
  display.clearDisplay();
  for (int i = 0; i < length; i++) {
    if (i == index) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  // Highlight selected
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, i * 10);
    display.print(items[i]);
  }
  display.display();
}