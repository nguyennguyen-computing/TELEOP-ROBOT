'use client'

import { useEffect, useState } from 'react'
import { cn } from '@/lib/utils'
import { Notification } from '@/types/robot'

export interface NotificationProps {
  notification: Notification
  onDismiss: (id: string) => void
  className?: string
}

const typeConfig = {
  success: {
    icon: '✓',
    bgColor: 'bg-success',
    textColor: 'text-success-foreground',
    borderColor: 'border-success',
  },
  warning: {
    icon: '⚠',
    bgColor: 'bg-warning',
    textColor: 'text-warning-foreground',
    borderColor: 'border-warning',
  },
  error: {
    icon: '✕',
    bgColor: 'bg-destructive',
    textColor: 'text-destructive-foreground',
    borderColor: 'border-destructive',
  },
  info: {
    icon: 'ℹ',
    bgColor: 'bg-primary',
    textColor: 'text-primary-foreground',
    borderColor: 'border-primary',
  },
}

export function NotificationItem({ 
  notification, 
  onDismiss, 
  className 
}: NotificationProps) {
  const [isVisible, setIsVisible] = useState(false)
  const [isExiting, setIsExiting] = useState(false)
  const config = typeConfig[notification.type]

  useEffect(() => {
    // Animate in
    const timer = setTimeout(() => setIsVisible(true), 10)
    
    // Auto dismiss if duration is set
    let dismissTimer: NodeJS.Timeout
    if (notification.duration && notification.duration > 0) {
      dismissTimer = setTimeout(() => {
        handleDismiss()
      }, notification.duration)
    }

    return () => {
      clearTimeout(timer)
      if (dismissTimer) clearTimeout(dismissTimer)
    }
    
    function handleDismiss() {
      setIsExiting(true)
      setTimeout(() => {
        onDismiss(notification.id)
      }, 200) // Match animation duration
    }
  }, [notification.duration, onDismiss])

  const handleDismiss = () => {
    setIsExiting(true)
    setTimeout(() => {
      onDismiss(notification.id)
    }, 200) // Match animation duration
  }

  return (
    <div
      className={cn(
        'flex items-start space-x-3 p-4 rounded-lg border shadow-lg transition-all duration-200 transform',
        config.bgColor,
        config.textColor,
        config.borderColor,
        {
          'translate-x-0 opacity-100': isVisible && !isExiting,
          'translate-x-full opacity-0': !isVisible || isExiting,
        },
        className
      )}
    >
      {/* Icon */}
      <div className="flex-shrink-0 w-5 h-5 flex items-center justify-center rounded-full bg-white/20 text-sm font-bold">
        {config.icon}
      </div>

      {/* Content */}
      <div className="flex-1 min-w-0">
        <p className="text-sm font-medium leading-tight">
          {notification.message}
        </p>
        <p className="text-xs opacity-80 mt-1">
          {notification.timestamp.toLocaleTimeString()}
        </p>
      </div>

      {/* Dismiss button */}
      <button
        onClick={handleDismiss}
        className="flex-shrink-0 w-5 h-5 flex items-center justify-center rounded-full hover:bg-white/20 transition-colors duration-150"
        aria-label="Dismiss notification"
      >
        <span className="text-xs">×</span>
      </button>
    </div>
  )
}

export interface NotificationContainerProps {
  notifications: Notification[]
  onDismiss: (id: string) => void
  position?: 'top-right' | 'top-left' | 'bottom-right' | 'bottom-left'
  className?: string
}

export function NotificationContainer({ 
  notifications, 
  onDismiss,
  position = 'top-right',
  className 
}: NotificationContainerProps) {
  const positionClasses = {
    'top-right': 'top-4 right-4',
    'top-left': 'top-4 left-4',
    'bottom-right': 'bottom-4 right-4',
    'bottom-left': 'bottom-4 left-4',
  }

  if (notifications.length === 0) return null

  return (
    <div 
      className={cn(
        'fixed z-50 flex flex-col space-y-2 max-w-sm w-full',
        positionClasses[position],
        className
      )}
    >
      {notifications.map((notification) => (
        <NotificationItem
          key={notification.id}
          notification={notification}
          onDismiss={onDismiss}
        />
      ))}
    </div>
  )
}

// Hook for managing notifications
export function useNotifications() {
  const [notifications, setNotifications] = useState<Notification[]>([])

  const addNotification = (
    type: Notification['type'],
    message: string,
    duration?: number
  ) => {
    const notification: Notification = {
      id: Math.random().toString(36).substr(2, 9),
      type,
      message,
      duration: duration ?? (type === 'error' ? 0 : 5000), // Errors stay until dismissed
      timestamp: new Date(),
    }

    setNotifications(prev => [...prev, notification])
    return notification.id
  }

  const dismissNotification = (id: string) => {
    setNotifications(prev => prev.filter(n => n.id !== id))
  }

  const clearAll = () => {
    setNotifications([])
  }

  return {
    notifications,
    addNotification,
    dismissNotification,
    clearAll,
  }
}