-- Sends simulated altitude status continuously

-- Define the severity levels
local STATUSTEXT_SEVERITY_EMERGENCY = 0
local STATUSTEXT_SEVERITY_ALERT     = 1
local STATUSTEXT_SEVERITY_CRITICAL  = 2
local STATUSTEXT_SEVERITY_ERROR     = 3
local STATUSTEXT_SEVERITY_WARNING   = 4
local STATUSTEXT_SEVERITY_NOTICE    = 5
local STATUSTEXT_SEVERITY_INFO      = 6
local STATUSTEXT_SEVERITY_DEBUG     = 7

local UPDATE_RATE_MS = 2000  
local simulated_alt = 0     
local direction = 1          


function update()
  simulated_alt = simulated_alt + (0.5 * direction)
  
  if simulated_alt > 50 then
    direction = -1 
  elseif simulated_alt < 0 then
    direction = 1 
  end

  local msg_string = string.format("RWLG - Simulated Altitude: %.1f m", simulated_alt)

  --Sends the message to the GCS
  gcs:send_text(STATUSTEXT_SEVERITY_INFO, msg_string)

  -- Tell ArduPilot to run this 'update' function again after the specified delay
  return update, UPDATE_RATE_MS
end

-- Send a startup message once when the script loads
gcs:send_text(STATUSTEXT_SEVERITY_INFO, "Status Sender script started (simulated mode).")
return update()