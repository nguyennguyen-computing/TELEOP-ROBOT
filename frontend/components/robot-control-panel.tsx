"use client";

import { useState, useCallback, useEffect, useRef } from "react";
import { ThemeToggle } from "./theme-toggle";
import {
  ControlPad,
  VelocityDisplay,
  SpeedLevelDisplay,
  ConnectionInfo,
  NotificationContainer,
  useNotifications,
} from "./ui";
import { Direction, RobotState, VelocityCommand } from "@/types/robot";
import { useApiClient, useWebSocketClient } from "@/hooks/use-api";
import { useAuth } from "./auth-provider";
import { env } from "@/lib/env";
import { cn } from "@/lib/utils";
import { wsEventDebouncer } from "@/lib/event-debouncer";

export function RobotControlPanel() {
  const { notifications, addNotification, dismissNotification } =
    useNotifications();
  const apiClient = useApiClient();
  const wsClient = useWebSocketClient();
  const { isAuthenticated, user } = useAuth();

  // Create refs to avoid dependency issues
  const apiClientRef = useRef(apiClient);
  const wsClientRef = useRef(wsClient);
  const addNotificationRef = useRef(addNotification);
  const lastApiCallRef = useRef<number>(0);
  const API_CALL_DEBOUNCE = 50; // 50ms debounce to prevent duplicate calls

  // Update refs when values change
  useEffect(() => {
    apiClientRef.current = apiClient;
    wsClientRef.current = wsClient;
    addNotificationRef.current = addNotification;
  });

  const [robotState, setRobotState] = useState<RobotState>({
    levels: { up: 0, down: 0, left: 0, right: 0 },
    velocity: { vx: 0, vy: 0 },
    connectionStatus: "disconnected",
    isHolding: {},
    theme: "light",
    notifications: [],
    isFullscreen: false,
    hapticFeedback: true,
    soundEnabled: false,
    lastCommandTime: null,
    commandHistory: [],
  });

  // Real-time connection metrics
  const [connectionMetrics, setConnectionMetrics] = useState({
    latency: 0,
    reconnectAttempts: 0,
    connectionTime: 0,
    lastHeartbeat: null as Date | null,
  });

  // Control configuration constants
  const VX_MAX = env.VX_MAX; // m/s - Maximum X velocity
  const VY_MAX = env.VY_MAX; // m/s - Maximum Y velocity
  const MAX_LEVEL = 10; // Maximum level for each direction
  const STEP_X = VX_MAX / MAX_LEVEL; // Velocity step per level
  const STEP_Y = VY_MAX / MAX_LEVEL; // Velocity step per level

  // Debug log to check values
  console.log('Velocity config:', { VX_MAX, VY_MAX, MAX_LEVEL, STEP_X, STEP_Y });
  const HOLD_FREQUENCY = 15; // Hz - Hold repeat frequency (10-20 Hz requirement)
  const HOLD_INTERVAL = Math.round(1000 / HOLD_FREQUENCY); // ms between hold repeats
  const HOLD_INITIAL_DELAY = 200; // ms - Initial delay before continuous increment

  const calculateVelocity = useCallback(
    (levels: typeof robotState.levels) => {
      const net_x_level = levels.up - levels.down;
      const net_y_level = levels.right - levels.left;

      const clamp = (value: number, min: number, max: number) =>
        Math.min(Math.max(value, min), max);

      const vx = clamp(net_x_level, -10, 10) * STEP_X;
      const vy = clamp(net_y_level, -10, 10) * STEP_Y;

      return { vx, vy };
    },
    [STEP_X, STEP_Y]
  );

  // Send velocity command to API
  const sendVelocityCommand = useCallback(
    async (
      vx: number,
      vy: number,
      levels: { up: number; down: number; left: number; right: number }
    ) => {
      if (!isAuthenticated) {
        addNotificationRef.current(
          "error",
          "Authentication required to send commands"
        );
        return;
      }

      // Debounce API calls to prevent duplicates
      const now = Date.now();
      if (now - lastApiCallRef.current < API_CALL_DEBOUNCE) {
        return;
      }
      lastApiCallRef.current = now;

      try {
        const command: VelocityCommand = {
          vx,
          vy,
          levels,
          timestamp: new Date(),
          source: "web",
        };

        // Update command history
        setRobotState((prev) => ({
          ...prev,
          commandHistory: [command, ...prev.commandHistory.slice(0, 9)], // Keep last 10 commands
        }));

        // Try WebSocket first, fallback to HTTP API
        let commandSent = false;
        
        if (wsClientRef.current.isConnected) {
          try {
            // Send message with correct structure for backend
            wsClientRef.current.send("set_vel", {
              vx,
              vy,
              levels,
              timestamp: new Date(),
              source: "web",
            });
            commandSent = true;
            console.log("Command sent via WebSocket");
          } catch (wsError) {
            console.warn(
              "WebSocket send failed, falling back to HTTP API:",
              wsError
            );
          }
        }

        // Only use HTTP API if WebSocket failed or not connected
        if (!commandSent) {
          const httpResponse = await apiClientRef.current.sendVelocityCommand(
            command
          );
          if (!httpResponse.ok) {
            throw new Error(httpResponse.error || "HTTP API failed");
          }
          console.log("Command sent via HTTP API");
        }
      } catch (error) {
        const message =
          error instanceof Error ? error.message : "Failed to send command";
        addNotificationRef.current("error", `Command failed: ${message}`);
      }
    },
    [isAuthenticated]
  ); // Only depend on isAuthenticated

  const handleDirectionPress = useCallback(
    (direction: Direction) => {
      setRobotState((prev) => {
        const newLevels = { ...prev.levels };
        // Clamp level to maximum
        if (newLevels[direction] < MAX_LEVEL) {
          newLevels[direction] = Math.min(newLevels[direction] + 1, MAX_LEVEL);
        }

        const velocity = calculateVelocity(newLevels);

        // Send velocity command to API
        sendVelocityCommand(velocity.vx, velocity.vy, newLevels);

        // Haptic feedback for mobile devices
        if (prev.hapticFeedback && "vibrate" in navigator) {
          navigator.vibrate(50);
        }

        return {
          ...prev,
          levels: newLevels,
          velocity,
          lastCommandTime: new Date(),
          isHolding: { ...prev.isHolding, [direction]: true },
        };
      });
    },
    [calculateVelocity, sendVelocityCommand]
  );

  const handleDirectionRelease = useCallback((direction: Direction) => {
    setRobotState((prev) => ({
      ...prev,
      isHolding: { ...prev.isHolding, [direction]: false },
    }));
  }, []);

  const handleDirectionHold = useCallback(
    (direction: Direction) => {
      setRobotState((prev) => {
        const newLevels = { ...prev.levels };
        // Only increment if not at maximum level
        if (newLevels[direction] < MAX_LEVEL) {
          newLevels[direction] = Math.min(newLevels[direction] + 1, MAX_LEVEL);

          const velocity = calculateVelocity(newLevels);

          // Send velocity command to API
          sendVelocityCommand(velocity.vx, velocity.vy, newLevels);

          // Lighter haptic feedback for hold increments
          if (prev.hapticFeedback && "vibrate" in navigator) {
            navigator.vibrate(25);
          }

          return {
            ...prev,
            levels: newLevels,
            velocity,
            lastCommandTime: new Date(),
            isHolding: { ...prev.isHolding, [direction]: true },
          };
        }

        return {
          ...prev,
          isHolding: { ...prev.isHolding, [direction]: true },
        };
      });
    },
    [calculateVelocity, sendVelocityCommand]
  );

  const handleStop = useCallback(() => {
    const newLevels = { up: 0, down: 0, left: 0, right: 0 };
    const velocity = calculateVelocity(newLevels);

    // Send stop command to API
    sendVelocityCommand(velocity.vx, velocity.vy, newLevels);

    setRobotState((prev) => {
      // Stronger haptic feedback for stop
      if (prev.hapticFeedback && "vibrate" in navigator) {
        navigator.vibrate([100, 50, 100]);
      }

      return {
        ...prev,
        levels: newLevels,
        velocity,
        lastCommandTime: new Date(),
        isHolding: {},
      };
    });

    addNotification("info", "Robot stopped - all levels reset to 0");
  }, [calculateVelocity, sendVelocityCommand, addNotification]);

  // Enhanced WebSocket connection management with real-time monitoring
  useEffect(() => {
    if (!isAuthenticated) {
      setRobotState((prev) => ({ ...prev, connectionStatus: "disconnected" }));
      setConnectionMetrics((prev) => ({
        ...prev,
        latency: 0,
        reconnectAttempts: 0,
      }));
      return;
    }

    // Define event handlers as stable references
    const handleConnectionStatus = (data: any) => {
      // Use event debouncer to prevent duplicates
      if (
        !wsEventDebouncer.shouldProcessConnectionStatus(
          data.status,
          data.source
        )
      ) {
        return; // Skip duplicate event
      }

      setRobotState((prev) => {
        // Only update if status actually changed
        if (prev.connectionStatus !== data.status) {
          return { ...prev, connectionStatus: data.status };
        }
        console.log("📝 Connection status unchanged:", data.status);
        return prev;
      });

      // Update connection metrics
      setConnectionMetrics((prev) => ({
        ...prev,
        reconnectAttempts: data.reconnectAttempt || 0,
        connectionTime: data.connectionTime || 0,
      }));

      // Show appropriate notifications based on connection status
      if (data.status === "connected") {
        const message = data.reconnectAttempt
          ? `Reconnected successfully (attempt ${data.reconnectAttempt})`
          : "Connected to robot successfully (WebSocket)";
        addNotificationRef.current("success", message);
      } else if (data.status === "disconnected") {
        // DEADMAN SWITCH: Reset robot state to stopped when disconnected
        setRobotState((prev) => ({
          ...prev,
          connectionStatus: "disconnected",
          levels: { up: 0, down: 0, left: 0, right: 0 },
          velocity: { vx: 0, vy: 0 },
          isHolding: {}
        }));
        
        if (data.finalAttempt) {
          addNotificationRef.current(
            "error",
            "Connection lost - Robot stopped for safety. Max reconnection attempts reached."
          );
        } else if (!data.wasManual) {
          addNotificationRef.current(
            "warning",
            "Connection lost - Robot stopped for safety. Attempting to reconnect..."
          );
        }
      } else if (data.status === "connecting" && data.reconnectAttempt) {
        addNotificationRef.current(
          "info",
          `Reconnecting... (attempt ${data.reconnectAttempt}/${data.maxAttempts})`
        );
      }
    };

    const handleVelocityUpdate = (data: any) => {
      // Use debouncer for velocity updates (shorter debounce time for real-time data)
      const velocityKey = `${data.vx}_${data.vy}_${JSON.stringify(
        data.levels
      )}`;
      if (
        !wsEventDebouncer.shouldProcess("velocity_update", data, velocityKey)
      ) {
        return; // Skip duplicate
      }

      setRobotState((prev) => ({
        ...prev,
        velocity: { vx: data.vx, vy: data.vy },
        levels: data.levels || prev.levels,
        lastCommandTime: new Date(data.timestamp),
      }));
    };

    const handleHeartbeat = (data: any) => {
      console.log("💓 WebSocket heartbeat received:", data);

      // Heartbeats can be frequent, use shorter debounce
      if (
        !wsEventDebouncer.shouldProcess(
          "heartbeat",
          data,
          `heartbeat_${data.latency}`
        )
      ) {
        return; // Skip duplicate
      }

      setConnectionMetrics((prev) => ({
        ...prev,
        latency: data.latency,
        lastHeartbeat: new Date(),
      }));
    };

    // Set up event listeners
    wsClientRef.current.on("connection_status", handleConnectionStatus);
    wsClientRef.current.on("velocity_update", handleVelocityUpdate);
    wsClientRef.current.on("heartbeat", handleHeartbeat);

    const connectWebSocket = async () => {
      try {
        console.log("🔌 Starting WebSocket connection...");
        setRobotState((prev) => ({ ...prev, connectionStatus: "connecting" }));

        await wsClientRef.current.connect();

        console.log("✅ WebSocket connected successfully");
        console.log("🔍 WebSocket connection details:", {
          isConnected: wsClientRef.current.isConnected,
          readyState: wsClientRef.current.webSocket?.readyState,
          url: wsClientRef.current.connectionUrl,
        });
      } catch (error) {
        console.warn(
          "❌ WebSocket connection failed, falling back to HTTP API:",
          error
        );

        // Fallback to HTTP API mode
        setRobotState((prev) => ({ ...prev, connectionStatus: "connected" }));
        addNotificationRef.current(
          "info",
          "Using HTTP API (WebSocket unavailable)"
        );
      }
    };

    // Add a small delay to avoid immediate connection attempts
    const timeoutId = setTimeout(connectWebSocket, 500);

    return () => {
      console.log("🧹 Cleaning up WebSocket connection and listeners...");
      clearTimeout(timeoutId);

      // Remove event listeners with the exact same function references
      wsClientRef.current.off("connection_status", handleConnectionStatus);
      wsClientRef.current.off("velocity_update", handleVelocityUpdate);
      wsClientRef.current.off("heartbeat", handleHeartbeat);

      // Clear event debouncer history
      wsEventDebouncer.clear();

      wsClientRef.current.disconnect();
    };
  }, [isAuthenticated]);

  // Manual connection toggle for testing (WebSocket)
  const toggleConnection = useCallback(() => {
    setRobotState((prev) => {
      if (prev.connectionStatus === "connected") {
        wsClientRef.current.disconnect();
        addNotificationRef.current("info", "Manually disconnected from robot");
        return { ...prev, connectionStatus: "disconnected" };
      } else {
        // Reset reconnection attempts for manual reconnection
        wsClientRef.current.resetReconnectionAttempts();

        wsClientRef.current
          .connect()
          .then(() => {
            // Connection status will be updated via the event listener
            console.log("Manual reconnection successful");
          })
          .catch((error) => {
            console.warn("Manual reconnection failed:", error);
            setRobotState((current) => ({
              ...current,
              connectionStatus: "connected",
            }));
            addNotificationRef.current(
              "info",
              "Using HTTP API (WebSocket unavailable)"
            );
          });
        return { ...prev, connectionStatus: "connecting" };
      }
    });
  }, []); // Remove all dependencies to prevent infinite loops

  // Refs for keyboard hold functionality
  const keyHoldIntervals = useRef<{ [key: string]: NodeJS.Timeout }>({});
  const keyHoldTimeouts = useRef<{ [key: string]: NodeJS.Timeout }>({});
  const keysPressed = useRef<Set<string>>(new Set());

  // Keyboard support (WASD keys) with hold functionality
  useEffect(() => {
    const keyMap: { [key: string]: Direction } = {
      w: "up",
      a: "left",
      s: "down",
      d: "right",
    };

    const startKeyHold = (direction: Direction) => {
      const key = Object.keys(keyMap).find((k) => keyMap[k] === direction);
      if (!key) return;

      // Initial press
      handleDirectionPress(direction);

      // Start continuous hold after initial delay
      keyHoldTimeouts.current[key] = setTimeout(() => {
        // Continue incrementing at configured frequency (10-20Hz requirement)
        keyHoldIntervals.current[key] = setInterval(() => {
          if (keysPressed.current.has(key)) {
            handleDirectionHold(direction);
          }
        }, HOLD_INTERVAL);
      }, HOLD_INITIAL_DELAY);
    };

    const stopKeyHold = (direction: Direction) => {
      const key = Object.keys(keyMap).find((k) => keyMap[k] === direction);
      if (!key) return;

      if (keyHoldTimeouts.current[key]) {
        clearTimeout(keyHoldTimeouts.current[key]);
        delete keyHoldTimeouts.current[key];
      }

      if (keyHoldIntervals.current[key]) {
        clearInterval(keyHoldIntervals.current[key]);
        delete keyHoldIntervals.current[key];
      }

      handleDirectionRelease(direction);
    };

    const handleKeyDown = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();

      // Prevent default for all our keys
      if (key === " " || keyMap[key]) {
        event.preventDefault();
      }

      if (key === " ") {
        handleStop();
        return;
      }

      if (keyMap[key] && !event.repeat && !keysPressed.current.has(key)) {
        keysPressed.current.add(key);
        setRobotState((prev) => ({
          ...prev,
          isHolding: { ...prev.isHolding, [keyMap[key]]: true },
        }));
        startKeyHold(keyMap[key]);
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();

      if (keyMap[key] && keysPressed.current.has(key)) {
        event.preventDefault();
        keysPressed.current.delete(key);
        setRobotState((prev) => ({
          ...prev,
          isHolding: { ...prev.isHolding, [keyMap[key]]: false },
        }));
        stopKeyHold(keyMap[key]);
      }
    };

    // Handle window blur to stop all key holds
    const handleWindowBlur = () => {
      keysPressed.current.forEach((key) => {
        if (keyMap[key]) {
          stopKeyHold(keyMap[key]);
        }
      });
      keysPressed.current.clear();
      setRobotState((prev) => ({
        ...prev,
        isHolding: {},
      }));
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    window.addEventListener("blur", handleWindowBlur);

    return () => {
      // Cleanup all intervals and timeouts
      Object.values(keyHoldIntervals.current).forEach(clearInterval);
      Object.values(keyHoldTimeouts.current).forEach(clearTimeout);

      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      window.removeEventListener("blur", handleWindowBlur);
    };
  }, [
    handleDirectionPress,
    handleDirectionHold,
    handleDirectionRelease,
    handleStop,
    HOLD_INTERVAL,
    HOLD_INITIAL_DELAY,
  ]);

  return (
    <div className="w-full max-w-6xl mx-auto space-y-6 p-4">
      {/* Header with connection status and theme toggle */}
      <div className="flex justify-between items-center">
        <div className="flex items-center space-x-4">
          <ConnectionInfo
            status={robotState.connectionStatus}
            {...(robotState.lastCommandTime && {
              lastUpdate: robotState.lastCommandTime,
            })}
            {...(robotState.connectionStatus === "connected" &&
              connectionMetrics.latency > 0 && {
                latency: connectionMetrics.latency,
              })}
          />
          <button
            onClick={toggleConnection}
            className="text-xs text-muted-foreground hover:text-foreground transition-colors"
          >
            (Click to toggle)
          </button>
          {isAuthenticated && user && (
            <div className="text-xs text-muted-foreground">
              Authenticated as:{" "}
              <span className="text-foreground font-medium">{user.id}</span>
              {!env.WS_ENABLED && (
                <span className="ml-2 text-yellow-600">
                  (WebSocket disabled)
                </span>
              )}
            </div>
          )}
        </div>
        <ThemeToggle />
      </div>

      {/* Main control area */}
      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        {/* Control Panel */}
        <div className="xl:col-span-2 bg-card border rounded-lg p-6">
          <h2 className="text-xl font-semibold mb-6">Robot Control</h2>
          <div className="flex justify-center">
            <ControlPad
              levels={robotState.levels}
              isHolding={robotState.isHolding}
              onDirectionPress={handleDirectionPress}
              onDirectionRelease={handleDirectionRelease}
              onDirectionHold={handleDirectionHold}
              onStop={handleStop}
              disabled={robotState.connectionStatus !== "connected"}
              holdFrequency={HOLD_FREQUENCY}
              holdInitialDelay={HOLD_INITIAL_DELAY}
            />
          </div>
        </div>

        {/* Status Panel */}
        <div className="space-y-6">
          {/* Velocity Display */}
          <div className="bg-card border rounded-lg p-6">
            <h2 className="text-xl font-semibold mb-4">Velocity</h2>
            <VelocityDisplay
              vx={robotState.velocity.vx}
              vy={robotState.velocity.vy}
            />
          </div>

          {/* Speed Levels */}
          <div className="bg-card border rounded-lg p-6">
            <h2 className="text-xl font-semibold mb-4">Speed Levels</h2>
            <SpeedLevelDisplay 
              levels={robotState.levels} 
              maxLevel={MAX_LEVEL} 
            />
          </div>

          {/* Real-time Connection Metrics */}
          {robotState.connectionStatus === "connected" && (
            <div className="bg-card border rounded-lg p-6">
              <h2 className="text-xl font-semibold mb-4">Connection Metrics</h2>
              <div className="space-y-3 text-sm">
                {connectionMetrics.latency > 0 && (
                  <div className="flex justify-between">
                    <span className="text-muted-foreground">Latency:</span>
                    <span
                      className={cn(
                        "font-mono font-medium",
                        connectionMetrics.latency < 100
                          ? "text-green-600"
                          : connectionMetrics.latency < 300
                          ? "text-yellow-600"
                          : "text-red-600"
                      )}
                    >
                      {connectionMetrics.latency}ms
                    </span>
                  </div>
                )}

                {connectionMetrics.lastHeartbeat && (
                  <div className="flex justify-between">
                    <span className="text-muted-foreground">
                      Last Heartbeat:
                    </span>
                    <span className="font-mono font-medium">
                      {connectionMetrics.lastHeartbeat.toLocaleTimeString()}
                    </span>
                  </div>
                )}

                {robotState.commandHistory.length > 0 && (
                  <div className="flex justify-between">
                    <span className="text-muted-foreground">
                      Commands Sent:
                    </span>
                    <span className="font-mono font-medium">
                      {robotState.commandHistory.length}
                    </span>
                  </div>
                )}

                <div className="flex justify-between">
                  <span className="text-muted-foreground">Protocol:</span>
                  <span className="font-medium text-green-600">WebSocket</span>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Instructions */}
      <div className="bg-muted/50 border rounded-lg p-4">
        <h3 className="font-medium mb-2">Instructions</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <h4 className="text-sm font-medium mb-2">Touch/Mouse Controls</h4>
            <ul className="text-sm text-muted-foreground space-y-1">
              <li>
                • Click directional buttons to increase speed levels (1-10)
              </li>
              <li>
                • Hold buttons for continuous increment at {HOLD_FREQUENCY}Hz
              </li>
              <li>• Use STOP button to reset all levels to 0</li>
              <li>• Touch/mobile friendly with haptic feedback</li>
            </ul>
          </div>
          <div>
            <h4 className="text-sm font-medium mb-2">Keyboard Controls</h4>
            <ul className="text-sm text-muted-foreground space-y-1">
              <li>
                • <kbd className="px-1 py-0.5 bg-muted rounded text-xs">W</kbd>{" "}
                Forward
              </li>
              <li>
                • <kbd className="px-1 py-0.5 bg-muted rounded text-xs">S</kbd>{" "}
                Backward
              </li>
              <li>
                • <kbd className="px-1 py-0.5 bg-muted rounded text-xs">A</kbd>{" "}
                Left
              </li>
              <li>
                • <kbd className="px-1 py-0.5 bg-muted rounded text-xs">D</kbd>{" "}
                Right
              </li>
              <li>
                •{" "}
                <kbd className="px-1 py-0.5 bg-muted rounded text-xs">
                  Space
                </kbd>{" "}
                Stop
              </li>
            </ul>
          </div>
        </div>
        <p className="text-xs text-muted-foreground mt-3">
          Velocity is calculated from net levels: Forward-Backward, Right-Left
        </p>
      </div>

      {/* Footer */}
      <div className="text-center text-sm text-muted-foreground">
        Web Teleop Robot Control Interface - UI Components Implemented
      </div>

      {/* Notifications */}
      <NotificationContainer
        notifications={notifications}
        onDismiss={dismissNotification}
        position="top-right"
      />
    </div>
  );
}
