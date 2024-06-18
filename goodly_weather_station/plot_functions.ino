void plotData(const char *var, float hours, String &currentDate, String &currentTime) {
  
  displayStatePrev = displayState;
  displayState = DisplayState::PLOT;

  File file = SD.open(DATA_LOG_FILE, FILE_READ);

  if (!file) {
    Serial.print("Error opening file ");
    Serial.println(DATA_LOG_FILE);
    return;
  }

  const int charWidth = 5;
  const int charHeight = 7;
  const int yLabelWidth = charWidth * 4 + 1;
  const int maxDataPoints = SCREEN_WIDTH - yLabelWidth;
  float data[maxDataPoints];
  int dataCount = 0;
  int varIndex = -1;

  DateTime now = DateTime(currentDate.substring(0, 4).toInt(),
                          currentDate.substring(5, 7).toInt(),
                          currentDate.substring(8, 10).toInt(),
                          currentTime.substring(0, 2).toInt(),
                          currentTime.substring(3, 5).toInt(),
                          currentTime.substring(6, 8).toInt());
  DateTime startTime = now - TimeSpan(hours * 3600);

  // Read the header line
  String headerLine = file.readStringUntil('\n');
  int commaCount = 0;
  int startIdx = 0;

  for (int i = 0; i < headerLine.length(); i++) {
    if (headerLine[i] == ',' || i == headerLine.length() - 1) {
      String header = headerLine.substring(startIdx, i == headerLine.length() - 1 ? i + 1 : i);
      if (header.equals(var)) {
        varIndex = commaCount;
        break;
      }
      commaCount++;
      startIdx = i + 1;
    }
  }

  if (varIndex == -1) {
    Serial.print("Variable ");
    Serial.print(var);
    Serial.println(" not found in header.");
    file.close();
    return;
  }

  // Read data lines
  while (file.available()) {
    String line = file.readStringUntil('\n');
    int commaCount = 0;
    int startIdx = 0;

    // Extract the date and time
    String dateStr = "";
    String timeStr = "";
    bool isDateFound = false;
    bool isTimeFound = false;
    for (int i = 0; i < line.length(); i++) {
      if (line[i] == ',' || i == line.length() - 1) {
        if (!isDateFound) {
          dateStr = line.substring(startIdx, i == line.length() - 1 ? i + 1 : i);
          isDateFound = true;
        } else if (!isTimeFound) {
          timeStr = line.substring(startIdx, i == line.length() - 1 ? i + 1 : i);
          isTimeFound = true;
        } else if (commaCount == varIndex) {
          String dataStr = line.substring(startIdx, i == line.length() - 1 ? i + 1 : i);
          float value = dataStr.toFloat();
          DateTime logTime = DateTime(dateStr.substring(0, 4).toInt(),
                                      dateStr.substring(5, 7).toInt(),
                                      dateStr.substring(8, 10).toInt(),
                                      timeStr.substring(0, 2).toInt(),
                                      timeStr.substring(3, 5).toInt(),
                                      timeStr.substring(6, 8).toInt());
          if (logTime >= startTime) {
            if (dataCount < maxDataPoints) {
              data[dataCount] = value;
              dataCount++;
            } else {
              // Shift data to the left to make room for new data
              for (int j = 0; j < maxDataPoints - 1; j++) {
                data[j] = data[j + 1];
              }
              data[maxDataPoints - 1] = value;
            }
          }
          break;
        }
        commaCount++;
        startIdx = i + 1;
      }
    }
  }

  file.close();

  // Find the minimum and maximum data values
  float minData = data[0];
  float maxData = data[0];
  for (int i = 1; i < dataCount; i++) {
    if (data[i] < minData) {
      minData = data[i];
    }
    if (data[i] > maxData) {
      maxData = data[i];
    }
  }

  // Plot the data
  display.clearDisplay();

  // Print min and max values to the left of the y-axis
  display.setCursor(0, 0);
  display.print((int)maxData);  // Print max value at the top
  display.setCursor(0, SCREEN_HEIGHT - charHeight);
  display.print((int)minData);  // Print min value at the bottom

  display.drawLine(yLabelWidth + 1, 0, yLabelWidth + 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);                                // Y-axis
  display.drawLine(yLabelWidth + 1, SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);  // X-axis

  for (int i = 0; i < dataCount; i++) {
    int x = yLabelWidth + 1 + i;
    int y = map_float(data[i], minData, maxData, SCREEN_HEIGHT - 2, 0.0);  // Reverse y-axis mapping
    display.drawPixel(x, y, SSD1306_WHITE);
  }

  display.display();
}

void incPlotTime() {
  if (plotHoursIdx < sizeof(plotHours)) {
    plotHoursIdx++;
    displayMessageSignHours(plotHours[plotHoursIdx]);
  }
}

void decPlotTime() {
  if (plotHoursIdx > 0) {
    plotHoursIdx--;
    displayMessageSignHours(plotHours[plotHoursIdx]);
  }
}
