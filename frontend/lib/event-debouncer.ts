/**
 * Event debouncer utility to prevent duplicate WebSocket events
 */

interface EventRecord {
  timestamp: number
  data: any
}

export class EventDebouncer {
  private eventHistory: Map<string, EventRecord> = new Map()
  private debounceTime: number

  constructor(debounceTimeMs: number = 1000) {
    this.debounceTime = debounceTimeMs
  }

  /**
   * Check if an event should be processed or is a duplicate
   * @param eventType - Type of event (e.g., 'connection_status')
   * @param eventData - Event data to compare
   * @param customKey - Optional custom key for more specific deduplication
   * @returns true if event should be processed, false if it's a duplicate
   */
  shouldProcess(eventType: string, eventData: any, customKey?: string): boolean {
    const key = customKey || `${eventType}_${JSON.stringify(eventData)}`
    const now = Date.now()
    const lastEvent = this.eventHistory.get(key)

    // If no previous event or enough time has passed
    if (!lastEvent || (now - lastEvent.timestamp) >= this.debounceTime) {
      this.eventHistory.set(key, { timestamp: now, data: eventData })
      return true
    }

    console.log(`Event debounced: ${eventType}`, {
      key,
      timeSinceLastEvent: now - lastEvent.timestamp,
      debounceTime: this.debounceTime
    })

    return false
  }

  /**
   * Check if a connection status event should be processed
   * Special handling for connection status to avoid duplicates
   */
  shouldProcessConnectionStatus(status: string, source?: string): boolean {
    const key = `connection_status_${status}`
    const now = Date.now()
    const lastEvent = this.eventHistory.get(key)

    // Allow immediate processing if status actually changed
    if (!lastEvent) {
      this.eventHistory.set(key, { timestamp: now, data: { status, source } })
      return true
    }

    const timeDiff = now - lastEvent.timestamp

    // If same status within debounce time, skip
    if (timeDiff < this.debounceTime) {
      console.log(`Connection status debounced: ${status}`, {
        source,
        timeSinceLastEvent: timeDiff,
        lastSource: lastEvent.data.source
      })
      return false
    }

    // Update timestamp and allow processing
    this.eventHistory.set(key, { timestamp: now, data: { status, source } })
    return true
  }

  /**
   * Clear event history for a specific event type or all events
   */
  clear(eventType?: string) {
    if (eventType) {
      // Clear all keys that start with the event type
      for (const key of this.eventHistory.keys()) {
        if (key.startsWith(eventType)) {
          this.eventHistory.delete(key)
        }
      }
    } else {
      this.eventHistory.clear()
    }
  }

  /**
   * Get statistics about debounced events
   */
  getStats() {
    const stats = {
      totalEvents: this.eventHistory.size,
      eventTypes: new Set<string>(),
      oldestEvent: 0,
      newestEvent: 0
    }

    for (const [key, record] of this.eventHistory.entries()) {
      const eventType = key.split('_')[0]
      stats.eventTypes.add(eventType)
      
      if (stats.oldestEvent === 0 || record.timestamp < stats.oldestEvent) {
        stats.oldestEvent = record.timestamp
      }
      
      if (record.timestamp > stats.newestEvent) {
        stats.newestEvent = record.timestamp
      }
    }

    return {
      ...stats,
      eventTypes: Array.from(stats.eventTypes),
      oldestEventAge: stats.oldestEvent ? Date.now() - stats.oldestEvent : 0,
      newestEventAge: stats.newestEvent ? Date.now() - stats.newestEvent : 0
    }
  }
}

// Global instance for WebSocket events
export const wsEventDebouncer = new EventDebouncer(1000) // 1 second debounce