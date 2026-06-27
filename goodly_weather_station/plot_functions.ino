void plotAxes(float hours, const char *title) {
  display.clearDisplay();

  // Print plot name
  display.setCursor(0, 10 * 2);
  display.print(title);

  // Print plot time
  display.setCursor(0, 10 * 4);
  float minutes = hours * 60;
  float days = hours / 24;
  float years = days / 365;
  if (hours < 1) {
    display.print(String(minutes, 0) + "m");
  } else if (days > 0.99 && years < 1) {
    display.print(String(days, 0) + "d");
  } else if (years > 0.99) {
    display.print(String(years, 0) + "y");
  } else {
    display.print(String(hours, 0) + "h");
  }

  // Draw axis lines
  display.drawLine(yLabelWidth + 2, SCREEN_MARGIN_TOP, yLabelWidth + 2, SCREEN_HEIGHT - 1, SSD1306_WHITE);   // Y-axis
  display.drawLine(yLabelWidth + 2, SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);  // X-axis

  // Print wait text
  display.setCursor(SCREEN_WIDTH / 2 - 12 * charWidth / 2 + 2 * charWidth, SCREEN_HEIGHT / 2);
  display.print("Loading data");

  // Update display
  display.display();
}

void initPlotBuffers(int maxDataPoints, float data[], float dataMax[], float dataMin[], unsigned int dataCount[]) {
  for (int i = 0; i < maxDataPoints; i++) {
    data[i] = INVALID_NUMBER;
    dataMax[i] = INVALID_NUMBER;
    dataMin[i] = INVALID_NUMBER;
    dataCount[i] = 0;
  }
}

void addPlotValue(unsigned int dataIdx, float value, float data[], float dataMax[], float dataMin[], unsigned int dataCount[]) {
  if (dataCount[dataIdx] == 0) {
    data[dataIdx] = 0;
    dataMax[dataIdx] = value;
    dataMin[dataIdx] = value;
  }
  dataCount[dataIdx]++;
  data[dataIdx] = data[dataIdx] + value;
  dataMax[dataIdx] = max(dataMax[dataIdx], value);
  dataMin[dataIdx] = min(dataMin[dataIdx], value);
}

void addPlotAggregate(unsigned int dataIdx, float avgValue, float minValue, float maxValue, unsigned int samples,
                      float data[], float dataMax[], float dataMin[], unsigned int dataCount[]) {
  if (samples == 0) {
    return;
  }

  if (dataCount[dataIdx] == 0) {
    data[dataIdx] = 0;
    dataMax[dataIdx] = maxValue;
    dataMin[dataIdx] = minValue;
  }

  dataCount[dataIdx] += samples;
  data[dataIdx] = data[dataIdx] + avgValue * (float)samples;
  dataMax[dataIdx] = max(dataMax[dataIdx], maxValue);
  dataMin[dataIdx] = min(dataMin[dataIdx], minValue);
}

bool finalizeAndRenderPlot(int maxDataPoints, float data[], float dataMax[], float dataMin[], unsigned int dataCount[]) {
  // Calculate average of plot points with more than one data point
  for (int i = 0; i < maxDataPoints; i++) {
    if (dataCount[i] > 1) {
      data[i] = data[i] / dataCount[i];
    }
  }

  // Find the minimum and maximum data values
  float minData = INVALID_NUMBER;
  float maxData = INVALID_NUMBER;

  for (int i = 0; i < maxDataPoints; i++) {
    if (plotUseRange) {
      if (minData == INVALID_NUMBER && dataCount[i] > 0) {
        minData = dataMin[i];
      }
      if (maxData == INVALID_NUMBER && dataCount[i] > 0) {
        maxData = dataMin[i];
      }
      if (dataMin[i] < minData && dataCount[i] > 0) {
        minData = dataMin[i];
      }
      if (dataMax[i] > maxData && dataCount[i] > 0) {
        maxData = dataMax[i];
      }
    } else {
      if (minData == INVALID_NUMBER && dataCount[i] > 0) {
        minData = data[i];
      }
      if (maxData == INVALID_NUMBER && dataCount[i] > 0) {
        maxData = data[i];
      }
      if (data[i] < minData && dataCount[i] > 0) {
        minData = data[i];
      }
      if (data[i] > maxData && dataCount[i] > 0) {
        maxData = data[i];
      }
    }
  }

  if (minData == INVALID_NUMBER || maxData == INVALID_NUMBER) {
    return false;
  }

  // Plot the data
  display.setTextColor(SSD1306_WHITE);

  // Print min and max values to the left of the y-axis
  display.setCursor(0, SCREEN_MARGIN_TOP);
  if (maxData > 100) {
    display.print(maxData, 0);
  } else {
    display.print(maxData, 1);
  }
  display.setCursor(0, SCREEN_HEIGHT - charHeight);
  if (minData > 100) {
    display.print(minData, 0);
  } else {
    display.print(minData, 1);
  }

  // Clear plot area
  display.fillRect(yLabelWidth + 3, 0, maxDataPoints, SCREEN_HEIGHT - 2, SSD1306_BLACK);

  // Plot data
  for (int i = 0; i < maxDataPoints; i++) {
    if (dataCount[i] > 0) {
      int x = yLabelWidth + 3 + i;
      if (plotUseRange) {
        int y1 = map_float(dataMin[i], minData, maxData, SCREEN_HEIGHT - 2, SCREEN_MARGIN_TOP);
        int y2 = map_float(dataMax[i], minData, maxData, SCREEN_HEIGHT - 2, SCREEN_MARGIN_TOP);
        display.drawLine(x, y1, x, y2, SSD1306_WHITE);
      } else {
        int y = map_float(data[i], minData, maxData, SCREEN_HEIGHT - 2, SCREEN_MARGIN_TOP);
        display.drawPixel(x, y, SSD1306_WHITE);
      }
    }
  }

  display.display();
  return true;
}

void plotData(const char *var, float hours, String &currentDate, String &currentTime, const char *title) {
  // Plot area is width - label - line
  const int maxDataPoints = SCREEN_WIDTH - yLabelWidth - 1;
  float data[maxDataPoints];
  float dataMax[maxDataPoints];
  float dataMin[maxDataPoints];
  unsigned int dataCount[maxDataPoints];

  initPlotBuffers(maxDataPoints, data, dataMax, dataMin, dataCount);

#if USE_SD
  File file = SD.open(DATA_LOG_FILE, FILE_READ);

  if (!file) {
    displayMessage("[Plot]Error opening ");
    displayMessage(DATA_LOG_FILE);
    return;
  }

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
      header.trim();
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

          if (logTime >= startTime && dataStr != "NaN") {

            // map(x, in_min, in_max, out_min, out_max)
            // Map log points to current time scale
            unsigned int dataTimeIdx = map(logTime_epoch, startTime_epoch, now_epoch, 0, maxDataPoints - 1);

            addPlotValue(dataTimeIdx, value, data, dataMax, dataMin, dataCount);
          }
        }
        commaCount++;
        startIdx = i + 1;
      }
    }
  }

  file.close();
#else
  if (logBufferCount == 0) {
    displayMessage("No log data");
    return;
  }

  unsigned long now_epoch;
  if (timeOk && timeClient.isTimeSet()) {
    now_epoch = timeClient.getEpochTime();
  } else {
    now_epoch = millis() / 1000;
  }
  unsigned long startTime_epoch = now_epoch - (unsigned long)(hours * 3600.0);

  if (logBufferCapacity == 0) {
    displayMessage("No log buffer");
    return;
  }

  for (unsigned int i = 0; i < logBufferCount; i++) {
    unsigned int idx = (logBufferHead + logBufferCapacity - logBufferCount + i) % logBufferCapacity;
    const LogEntry &entry = logBuffer[idx];

    if (entry.epoch < startTime_epoch || entry.epoch > now_epoch) {
      continue;
    }

    float value = INVALID_NUMBER;
    float minValue = INVALID_NUMBER;
    float maxValue = INVALID_NUMBER;
    unsigned int samples = 0;
    bool valueOk = false;

    if (strcmp(var, "TempIn") == 0) {
      value = entry.tempIn;
      minValue = entry.tempInMin;
      maxValue = entry.tempInMax;
      samples = entry.tempInSamples;
      valueOk = entry.tempInOk && entry.tempInSamples > 0;
    } else if (strcmp(var, "TempOut") == 0) {
      value = entry.tempOut;
      minValue = entry.tempOutMin;
      maxValue = entry.tempOutMax;
      samples = entry.tempOutSamples;
      valueOk = entry.tempOutOk && entry.tempOutSamples > 0;
    } else if (strcmp(var, "PressureIn") == 0) {
      value = entry.pres;
      minValue = entry.presMin;
      maxValue = entry.presMax;
      samples = entry.presSamples;
      valueOk = entry.presOk && entry.presSamples > 0;
    } else if (strcmp(var, "HumidityIn") == 0) {
      value = entry.hum;
      minValue = entry.humMin;
      maxValue = entry.humMax;
      samples = entry.humSamples;
      valueOk = entry.humOk && entry.humSamples > 0;
    } else if (strcmp(var, "TempPcb") == 0) {
      value = entry.tempPcb;
      minValue = entry.tempPcbMin;
      maxValue = entry.tempPcbMax;
      samples = entry.tempPcbSamples;
      valueOk = entry.tempPcbOk && entry.tempPcbSamples > 0;
    }

    if (!valueOk && strcmp(var, "TempIn") != 0 && strcmp(var, "TempOut") != 0 && strcmp(var, "PressureIn") != 0 && strcmp(var, "HumidityIn") != 0 && strcmp(var, "TempPcb") != 0) {
      Serial.print("Variable ");
      Serial.print(var);
      Serial.println(" not found in RAM log.");
      return;
    }

    if (!valueOk) {
      continue;
    }

    unsigned int dataTimeIdx = maxDataPoints - 1;
    if (now_epoch > startTime_epoch) {
      dataTimeIdx = map(entry.epoch, startTime_epoch, now_epoch, 0, maxDataPoints - 1);
      if (dataTimeIdx >= (unsigned int)maxDataPoints) {
        dataTimeIdx = maxDataPoints - 1;
      }
    }

    addPlotAggregate(dataTimeIdx, value, minValue, maxValue, samples, data, dataMax, dataMin, dataCount);
  }
#endif

  if (!finalizeAndRenderPlot(maxDataPoints, data, dataMax, dataMin, dataCount)) {
    displayMessage("No valid log data");
  }
}

void incPlotTime() {
  if (plotHoursIdx < sizeof(plotHours) / sizeof(plotHours[0]) - 1) {
    plotHoursIdx++;
    savePlotSettings();
    //displayMessageSignHours(plotHours[plotHoursIdx]);
  }
}

void decPlotTime() {
  if (plotHoursIdx > 0) {
    plotHoursIdx--;
    savePlotSettings();
    //displayMessageSignHours(plotHours[plotHoursIdx]);
  }
}
