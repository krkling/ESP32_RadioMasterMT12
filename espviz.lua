-- /SCRIPTS/TELEMETRY/ESPViz.lua
-- Full-screen 128x64 monochrome telemetry console for RadioMaster MT12
-- Features: Top ST/TH/BR bar, 25-line scrollable history log, bottom TX/SIG bar,
-- and 0x2D backchannel push to XIAO ESP32C3.

local logLines = {}
local maxHistory = 25   -- Total messages kept in RAM
local visibleRows = 4   -- Rows visible between top and bottom bars
local scrollOffset = 0  -- 0 = live bottom; >0 = lines scrolled up into history

local lastFmText = ""
local lastPushTime = 0

local function getTxBatteryInfo()
  local v = getValue("tx-voltage") or 0
  local settings = getGeneralSettings()
  local vMin = (settings and settings.battMin) or 6.6
  local vMax = (settings and settings.battMax) or 8.4

  local pct = 0
  if vMax > vMin then
    pct = math.floor(((v - vMin) / (vMax - vMin)) * 100 + 0.5)
    if pct < 0 then pct = 0 end
    if pct > 100 then pct = 100 end
  end

  local vTenths = math.floor(v * 10 + 0.5)
  local vWhole = math.floor(vTenths / 10)
  local vDec = vTenths % 10
  return pct, vWhole, vDec, vTenths
end

local function pushLog(msg)
  if msg == nil or msg == "" then return end

  local count = #logLines
  if count < maxHistory then
    logLines[count + 1] = msg
  else
    for i = 1, maxHistory - 1 do
      logLines[i] = logLines[i + 1]
    end
    logLines[maxHistory] = msg
  end

  -- If the user is currently scrolled up reading history, keep their view pinned
  -- to the same lines as new messages arrive at the bottom.
  if scrollOffset > 0 then
    local maxOffset = math.max(0, #logLines - visibleRows)
    scrollOffset = math.min(maxOffset, scrollOffset + 1)
  end
end

local function background()
  local fm = getValue("FM")
  if type(fm) == "string" and fm ~= "" and fm ~= lastFmText then
    lastFmText = fm
    pushLog(fm)
  end

  local command, data = crossfireTelemetryPop()
  if command ~= nil and data ~= nil then
    pushLog(string.format("CMD 0x%02X len=%d", command, #data))
  end

  local now = getTime()
  if now - lastPushTime >= 50 then
    lastPushTime = now
    local txBatPct, _, _, vTenths = getTxBatteryInfo()
    local sigPct = math.floor((getValue("RQly") or 0) + 0.5)
    crossfireTelemetryPush(0x2D, { 0xEE, 0xEA, txBatPct, sigPct, vTenths })
  end
end

local function run(event)
  local count = #logLines
  local maxOffset = math.max(0, count - visibleRows)

  -- Handle MT12 Scroll Wheel & Click events
  if event == EVT_VIRTUAL_PREV then
    -- Scroll UP into older messages
    if scrollOffset < maxOffset then
      scrollOffset = scrollOffset + 1
    end
  elseif event == EVT_VIRTUAL_NEXT then
    -- Scroll DOWN toward newest messages (0 = resumes live auto-scroll)
    if scrollOffset > 0 then
      scrollOffset = scrollOffset - 1
    end
  elseif event == EVT_VIRTUAL_ENTER then
    -- Click scroll wheel to immediately snap back to live bottom
    scrollOffset = 0
  end

  lcd.clear()

  -- 1. Top Bar: Steering, Throttle, Brake
  local rxBt = getValue("RxBt") or 100
  local curr = getValue("Curr") or 0
  local batP = getValue("Bat%") or 0

  local strPct = math.floor(rxBt - 100 + 0.5)
  local thrPct = math.floor(curr + 0.5)
  local brkPct = math.floor(batP + 0.5)

  lcd.drawText(1,  1, string.format("ST:%+4d%%", strPct), SMLSIZE)
  lcd.drawText(47, 1, string.format("TH:%3d%%",  thrPct), SMLSIZE)
  lcd.drawText(89, 1, string.format("BR:%3d%%",  brkPct), SMLSIZE)
  lcd.drawLine(0, 9, 127, 9, SOLID, FORCE)

  -- 2. Middle Scrollable Console Area (y = 11..51, 4 lines)
  if count == 0 then
    lcd.drawText(4, 26, "Waiting for ESP32...", SMLSIZE)
  else
    local endIdx = count - scrollOffset
    local startIdx = math.max(1, endIdx - visibleRows + 1)

    local row = 0
    for i = startIdx, endIdx do
      local y = 11 + row * 10
      -- Highlight only the newest message in the entire buffer when it is visible
      local flags = (i == count) and (SMLSIZE + INVERS) or SMLSIZE
      lcd.drawText(1, y, logLines[i], flags)
      row = row + 1
    end

    -- Draw a compact scroll indicator on the far right when viewing history
    if scrollOffset > 0 then
      lcd.drawText(116, 11, string.format("-%d", scrollOffset), SMLSIZE + INVERS)
    end
  end

  -- 3. Bottom Status Bar: MT12 Battery % + Voltage and Signal Strength %
  local txBatPct, vWhole, vDec = getTxBatteryInfo()
  local sigPct = math.floor((getValue("RQly") or 0) + 0.5)

  lcd.drawLine(0, 53, 127, 53, SOLID, FORCE)
  lcd.drawText(1,  55, string.format("TX:%3d%%(%d.%dV)", txBatPct, vWhole, vDec), SMLSIZE)
  lcd.drawText(76, 55, string.format("SIG:%3d%%", sigPct), SMLSIZE)
end

return { run = run, background = background }