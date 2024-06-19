void plotData(const char *var, float hours, String &currentDate, String &currentTime) {

  //displayState = DisplayState::PLOT;

  File file = SD.open(DATA_LOG_FILE, FILE_READ);

  if (!file) {
    displayMessage("[Plot]Error opening ");
    displayMessage(DATA_LOG_FILE);
    return;
  }

  const int charWidth = 5;
  const int charHeight = 7;
  // y-axis label (characters plus margin)
  const int yLabelWidth = charWidth * 4 + 1;
  // Plot area is width - label - line
  const int maxDataPoints = SCREEN_WIDTH - yLabelWidth - 1;
  float data[maxDataPoints];
  for (int i = 0; i < maxDataPoints; i++) {
    data[i] = INVALID_NUMBER;
  }
  //unsigned int dataCount = 0;
  unsigned int dataIdx = 0;
  unsigned int dataAvgCount[maxDataPoints] = { 0 };
  int varIndex = -1;

  DateTime now = DateTime(currentDate.substring(0, 4).toInt(),
                          currentDate.substring(5, 7).toInt(),
                          currentDate.substring(8, 10).toInt(),
                          currentTime.substring(0, 2).toInt(),
                          currentTime.substring(3, 5).toInt(),
                          currentTime.substring(6, 8).toInt());
  DateTime startTime = now - TimeSpan(hours * 3600);

  unsigned int now_epoch = now.unixtime();
  unsigned int startTime_epoch = startTime.unixtime();

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

          unsigned int logTime_epoch = logTime.unixtime();

          if (logTime >= startTime) {

            // map(x, in_min, in_max, out_min, out_max)
            // Map log points to current time scale
            unsigned int dataIdx = map(logTime_epoch, startTime_epoch, now_epoch, 0, maxDataPoints - 1);

            // Add data value to plot time index
            if (dataAvgCount[dataIdx] == 0) {
              data[dataIdx] = 0;  // Replace INVALID_NUMBER with 0
            }
            dataAvgCount[dataIdx]++;
            data[dataIdx] = data[dataIdx] + value;
          }
        }
        commaCount++;
        startIdx = i + 1;
      }
    }
  }

  file.close();

  // Calculate average of plot points with more than one data point
  for (int i = 0; i < maxDataPoints; i++) {
    if (dataAvgCount[i] > 1) {
      data[i] = data[i] / dataAvgCount[i];
    }
  }

  // Find the minimum and maximum data values
  float minData = INVALID_NUMBER;
  float maxData = INVALID_NUMBER;
  for (int i = 0; i < maxDataPoints; i++) {
    if (minData == INVALID_NUMBER && dataAvgCount[i] > 0) {
      minData = data[i];
    }
    if (maxData == INVALID_NUMBER && dataAvgCount[i] > 0) {
      maxData = data[i];
    }
    if (data[i] < minData && dataAvgCount[i] > 0) {
      minData = data[i];
    }
    if (data[i] > maxData && dataAvgCount[i] > 0) {
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

  // Draw axis lines
  display.drawLine(yLabelWidth + 2, 0, yLabelWidth + 2, SCREEN_HEIGHT - 1, SSD1306_WHITE);                   // Y-axis
  display.drawLine(yLabelWidth + 2, SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);  // X-axis

  // Plot data
  for (int i = 0; i < maxDataPoints; i++) {
    if (dataAvgCount[i] > 0) {
      int x = yLabelWidth + 2 + i;
      int y = map_float(data[i], minData, maxData, SCREEN_HEIGHT - 2, 0.0);  // Reverse y-axis mapping
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }

  display.display();
}

void incPlotTime() {
  if (plotHoursIdx < sizeof(plotHours) / sizeof(plotHours[0]) - 1) {
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
