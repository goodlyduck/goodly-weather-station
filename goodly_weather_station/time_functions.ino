unsigned long toEpoch(String& date, String& time) {
  // Parse the date string in the format "YYYY-MM-DD"
  int year = date.substring(0, 4).toInt();
  int month = date.substring(5, 7).toInt();
  int day = date.substring(8, 10).toInt();
  
  // Parse the time string in the format "HH:MM:SS"
  int hour = time.substring(0, 2).toInt();
  int minute = time.substring(3, 5).toInt();
  int second = time.substring(6, 8).toInt();
  
  // Create a DateTime object
  DateTime dateTime(year, month, day, hour, minute, second);
  
  // Return the epoch time
  return dateTime.unixtime();
}