-- /SCRIPTS/TELEMETRY/console.lua
-- Full-screen 128x64 monochrome telemetry console for RadioMaster MT12

local logLines = {}
local maxLines = 5
local lastFmText = ""
local msgCounter = 0

-- Push a new message into the rolling log buffer
local function pushLog(msg)
  if msg == nil or msg == "" then return end
  msgCounter = msgCounter + 1
  local entry = string.format("%02d:%s", msgCounter % 100, msg)
  table.insert(logLines, entry)
  if #logLines > maxLines then
    table.remove(logLines, 1)
  end
end

-- Runs continuously in the background even when not viewing the TELE screen
local function background()
  -- 1. Poll Flight Mode string (0x21) for new incoming text messages
  local fm = getValue("FM")
  if type(fm) == "string" and fm ~= "" and fm ~= lastFmText then
    lastFmText = fm
    pushLog(fm)
  end

  -- 2. Also listen for raw CRSF extended frames (0x2D / 0x32) for structured commands
  local command, data = crossfireTelemetryPop()
  if command ~= nil and data ~= nil then
    -- Example: catch custom payload and log its command byte + length
    pushLog(string.format("CMD 0x%02X len=%d", command, #data))
  end
end

-- Draws the 128x64 screen when the TELE button is active
local function run(event)
  lcd.clear()

  -- Read normalized numeric channels from 0x08 Battery frame (0.0 to 200.0 -> -100% to +100%)
  local rxBt = getValue("RxBt") or 0
  local curr = getValue("Curr") or 0
  local ch1Pct = math.floor(rxBt - 100) -- Maps 0..200V to -100..+100%
  local ch2Pct = math.floor(curr - 100) -- Maps 0..200A to -100..+100%

  -- Top Header Bar (y = 0..9): Live CH1 (RxBt) and CH2 (Curr) values
  lcd.drawText(1, 1, string.format("CH1:%+4d%%", ch1Pct), SMLSIZE)
  lcd.drawText(66, 1, string.format("CH2:%+4d%%", ch2Pct), SMLSIZE)
  lcd.drawLine(0, 9, 127, 9, SOLID, FORCE)

  -- Scrolling Console Area (y = 11..63, 5 lines at 10px spacing)
  if #logLines == 0 then
    lcd.drawText(4, 28, "Waiting for ESP32...", SMLSIZE)
  else
    for i = 1, #logLines do
      local y = 11 + (i - 1) * 10
      -- Highlight the newest message at the bottom in inverse video
      local flags = (i == #logLines) and (SMLSIZE + INVERS) or SMLSIZE
      lcd.drawText(1, y, logLines[i], flags)
    end
  end
end

return { run = run, background = background }