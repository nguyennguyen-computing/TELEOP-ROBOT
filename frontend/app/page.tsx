import { RobotControlPanel } from '@/components/robot-control-panel'
import { AuthGuard } from '@/components/auth-guard'
import { UserProfile } from '@/components/ui/user-profile'

export default function Home() {
  return (
    <AuthGuard requiredScopes={['robot:control']}>
      <main className="container mx-auto px-4 py-8">
        <div className="max-w-4xl mx-auto">
          <header className="text-center mb-8">
            <h1 className="text-4xl font-bold text-primary mb-2">
              Web Teleop Robot
            </h1>
            <p className="text-muted-foreground">
              Remote robot control interface
            </p>
          </header>
          
          {/* User profile section */}
          <div className="mb-6">
            <UserProfile />
          </div>
          
          <RobotControlPanel />
        </div>
      </main>
    </AuthGuard>
  )
}