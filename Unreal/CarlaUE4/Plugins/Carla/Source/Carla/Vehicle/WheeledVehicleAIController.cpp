// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "WheeledVehicleAIController.h"

#include "MapGen/RoadMap.h"
#include "Traffic/RoutePlanner.h"
#include "Vehicle/CarlaWheeledVehicle.h"

#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "WheeledVehicleMovementComponent.h"

#include <algorithm>

// =============================================================================
// -- Static local methods -----------------------------------------------------
// =============================================================================

static bool RayCast(const AActor &Actor, const FVector &Start, const FVector &End)
{
  // UE_LOG(LogCarla, Warning, TEXT("********* RayCast() **********"));

  FHitResult OutHit;
  static FName TraceTag = FName(TEXT("VehicleTrace"));
  FCollisionQueryParams CollisionParams(TraceTag, true);
  CollisionParams.AddIgnoredActor(&Actor);

  const bool Success = Actor.GetWorld()->LineTraceSingleByObjectType(
      OutHit,
      Start,
      End,
      FCollisionObjectQueryParams(FCollisionObjectQueryParams::AllDynamicObjects),
      CollisionParams);

  // DrawDebugLine(Actor.GetWorld(), Start, End,
  //     Success ? FColor(255, 0, 0) : FColor(0, 255, 0), false);

  return Success && OutHit.bBlockingHit;
}

static bool IsThereAnObstacleAhead(
    const ACarlaWheeledVehicle &Vehicle,
    const float Speed,
    const FVector &Direction)
{
  // UE_LOG(LogCarla, Warning, TEXT("********* IsThereAnObstacleAhead() **********"));

  const auto ForwardVector = Vehicle.GetVehicleOrientation();
  const auto VehicleBounds = Vehicle.GetVehicleBoundingBoxExtent();

  FVector NormDirection = Direction.GetSafeNormal();

  const float Distance = std::max(50.0f, Speed * Speed); // why?

  const FVector StartCenter = Vehicle.GetActorLocation() +
      (ForwardVector * (250.0f + VehicleBounds.X / 2.0f)) + FVector(0.0f, 0.0f, 50.0f);
  const FVector EndCenter = StartCenter + NormDirection * (Distance + VehicleBounds.X / 2.0f);

  const FVector StartRight = StartCenter +
      (FVector(ForwardVector.Y, -ForwardVector.X, ForwardVector.Z) * 100.0f);
  const FVector EndRight = StartRight + NormDirection * (Distance + VehicleBounds.X / 2.0f);

  const FVector StartLeft = StartCenter +
      (FVector(-ForwardVector.Y, ForwardVector.X, ForwardVector.Z) * 100.0f);
  const FVector EndLeft = StartLeft + NormDirection * (Distance + VehicleBounds.X / 2.0f);

  return
    RayCast(Vehicle, StartCenter, EndCenter) ||
    RayCast(Vehicle, StartRight, EndRight) ||
    RayCast(Vehicle, StartLeft, EndLeft);
}

template <typename T>
static void ClearQueue(std::queue<T> &Queue)
{
  UE_LOG(LogCarla, Warning, TEXT("********* ClearQueue() **********"));

  std::queue<T> EmptyQueue;
  Queue.swap(EmptyQueue);
}

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

AWheeledVehicleAIController::AWheeledVehicleAIController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  // UE_LOG(LogCarla, Warning, TEXT("********* AWheeledVehicleAIController() **********"));

  last_AutopilotControl.Brake = 0.0f;
  last_AutopilotControl.Throttle = 0.0f;
  last_AutopilotControl.Steer = 0.0f;

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));

  RandomEngine->Seed(RandomEngine->GenerateRandomSeed());

  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;

  // UE_LOG(LogCarla, Warning, TEXT("------------ set fixed route in constructor -----------"));
  // TArray<FVector> Locations;
  // SetFixedRoute(Locations);
}

AWheeledVehicleAIController::~AWheeledVehicleAIController() {}

// =============================================================================
// -- AController --------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::OnPossess(APawn *aPawn)
{
  UE_LOG(LogCarla, Warning, TEXT("********* OnPossess() **********"));

  Super::OnPossess(aPawn);

  if (IsPossessingAVehicle())
  {
    UE_LOG(LogCarla, Error, TEXT("Controller already possessing a vehicle!"));
    return;
  }
  Vehicle = Cast<ACarlaWheeledVehicle>(aPawn);
  check(Vehicle != nullptr);
  MaximumSteerAngle = Vehicle->GetMaximumSteerAngle();
  check(MaximumSteerAngle > 0.0f);
  ConfigureAutopilot(bAutopilotEnabled);

  if (RoadMap == nullptr)
  {
    TActorIterator<ACityMapGenerator> It(GetWorld());
    RoadMap = (It ? It->GetRoadMap() : nullptr);
  }
  if (RoadMap == nullptr) {
  UE_LOG(LogCarla, Warning, TEXT("********* OnPossess(), RoadMap is nullptr **********"));
    
  } else {
  UE_LOG(LogCarla, Warning, TEXT("********* OnPossess(), RoadMap is initialized **********"));

  }
}

void AWheeledVehicleAIController::OnUnPossess()
{
  UE_LOG(LogCarla, Warning, TEXT("********* OnUnPossess() **********"));

  Super::OnUnPossess();

  Vehicle = nullptr;
}

void AWheeledVehicleAIController::Tick(const float DeltaTime)
{
  // UE_LOG(LogCarla, Warning, TEXT("********* Tick() **********"));

  Super::Tick(DeltaTime);

  if (!IsPossessingAVehicle())
  {
    return;
  }

  if (bAutopilotEnabled)
  {
    Vehicle->ApplyVehicleControl(TickAutopilotController(), EVehicleInputPriority::Autopilot);
  }
  else if (!bControlIsSticky)
  {
    Vehicle->ApplyVehicleControl(FVehicleControl{}, EVehicleInputPriority::Relaxation);
  }

  Vehicle->FlushVehicleControl();
}

// =============================================================================
// -- Autopilot ----------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::ConfigureAutopilot(const bool Enable, const bool KeepState)
{
  UE_LOG(LogCarla, Warning, TEXT("********* ConfigureAutopilot() **********"));

  bAutopilotEnabled = Enable;
  if (!KeepState)
  {
    // Reset state.
    Vehicle->SetSteeringInput(0.0f);
    Vehicle->SetThrottleInput(0.0f);
    Vehicle->SetBrakeInput(0.0f);
    Vehicle->SetReverse(false);
    Vehicle->SetHandbrakeInput(false);
    // ClearQueue(TargetLocations);
    Vehicle->SetAIVehicleState(
        bAutopilotEnabled ?
        ECarlaWheeledVehicleState::FreeDriving :
        ECarlaWheeledVehicleState::AutopilotOff);
  }

  TrafficLightState = ETrafficLightState::Green;

  /// @todo Workaround for a race condition between client and server when
  /// enabling autopilot right after initializing a vehicle.
  if (bAutopilotEnabled)
  {
    for (TActorIterator<ARoutePlanner> It(GetWorld()); It; ++It)
    {
      ARoutePlanner *RoutePlanner = *It;
      // Check if we are inside this route planner.
      TSet<AActor *> OverlappingActors;
      RoutePlanner->TriggerVolume->GetOverlappingActors(
          OverlappingActors,
          ACarlaWheeledVehicle::StaticClass());
      for (auto *Actor : OverlappingActors)
      {
        if (Actor == Vehicle)
        {
          RoutePlanner->AssignRandomRoute(*this);
          return;
        }
      }
    }
  }
}

// =============================================================================
// -- Traffic ------------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::SetFixedRoute(
    const TArray<FVector> &Locations,
    const bool bOverwriteCurrent)
{
  /*
  // UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRoute() **********"));
  // if (bOverwriteCurrent)
  // {
  //   ClearQueue(TargetLocations);
  // }
  // for (auto &Location : Locations)
  // {
  //   TargetLocations.emplace(Location);
  // }
  if(!TargetLocations.empty()) {
    UE_LOG(LogCarla, Error, TEXT("*********  can't SetFixedRoute() because TargetLocations is not empty **********"));
    return;
  }
  // 直行
  // FVector loc(906.55, 12885.20, 120);
  // TargetLocations.emplace(loc);
  // loc = FVector(918.56, 15265.16, 120);
  // TargetLocations.emplace(loc);
  // loc = FVector(934.70, 18465.13, 120);
  // TargetLocations.emplace(loc);
  // loc = FVector(948.83, 21265.09, 120);
  // TargetLocations.emplace(loc);
  // 跨越车道
  FVector loc(20096.1, -39580.89, 120);
  TargetLocations.emplace(loc);
  // 转弯
  // FVector loc(31048.72, -11297.29, 120);
  // TargetLocations.emplace(loc);
  UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRoute() Done!!! **********"));
  UE_LOG(LogCarla, Warning, TEXT("999, TargetLocations.size() = %d"), static_cast<int>(TargetLocations.size()));
*/
}

void AWheeledVehicleAIController::SetFixedRouteAll(const TArray<FVector> &locs) {
  UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRouteAll() **********"));
  for(const auto& pt : locs) {
    TargetLocations.emplace(pt);
  }
}

void AWheeledVehicleAIController::SetFixedRouteOnePoint(float x, float y, float z) {
  UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRouteOnePoint() **********"));
  TargetLocations.emplace(FVector(x, y, 0));
  speeds.emplace(z);
  SpeedLimit = speeds.front();
  /*
  const auto CurrentLocation = [&]() {
    const auto &Wheels = Vehicle->GetVehicleMovementComponent()->Wheels;
    check((Wheels.Num() > 1) && (Wheels[0u] != nullptr) && (Wheels[1u] != nullptr));
    return (Wheels[0u]->Location + Wheels[1u]->Location) / 2.0f;
  } ();
  std::cout << "CurrentLocation = (" << CurrentLocation.X << ", " << CurrentLocation.Y << ")"<< std::endl;
  std::cout << "SetRoutePoint   = (" << x << ", " << y << ")"<< std::endl;

  if(TargetLocations.size() > 35 && TargetLocations.size() < 60) {
    auto temp_route = TargetLocations;
    int cnt = static_cast<int>(temp_route.size());
    auto temp_speeds = speeds;
    for(int i=0; i<cnt; i++) {
      auto loc = temp_route.front();
      temp_route.pop();
      float spe = -111.11;
      if(!temp_speeds.empty()) {
        spe = temp_speeds.front();
        temp_speeds.pop();
      }  
      std::cout << "Route " << i << " -- (" << loc.X << ", " << loc.Y << ", " << loc.Z << "), v = " << spe  << " km/h" << std::endl;
    }
  } else {
    std::cout << "TargetLocations.size() is beyond range, = " << TargetLocations.size() << std::endl;
  }*/
}

void AWheeledVehicleAIController::ClearFixedRoute() {
  UE_LOG(LogCarla, Warning, TEXT("********* ClearFixedRoute() **********"));
  ClearQueue(TargetLocations);
  ClearQueue(speeds);
}

// =============================================================================
// -- AI -----------------------------------------------------------------------
// =============================================================================

FVehicleControl AWheeledVehicleAIController::TickAutopilotController()
{
  // SetSpeedLimit(17.8);
  // UE_LOG(LogCarla, Warning, TEXT("********* TickAutopilotController(), SpeedLimit = %f km/h **********"), SpeedLimit);


#if WITH_EDITOR // This happens in simulation mode in editor.
  if (Vehicle == nullptr)
  {
    bAutopilotEnabled = false;
    return {};
  }
#endif // WITH_EDITOR

  check(Vehicle != nullptr);
  FVehicleControl AutopilotControl;

  if (TargetLocations.empty())
  {
    if(isOpenLog) {
      UE_LOG(LogCarla, Warning, TEXT("TargetLocations.size() = %d, path empty, car should stop **********"),
             static_cast<int>(TargetLocations.size()));  
    }
    isOpenLog = false;
    // AutopilotControl.Brake = 0.0f;
    // AutopilotControl.Throttle = 0.0f;
    // AutopilotControl.Steer = 0.0f;
    return last_AutopilotControl;
  }

  FVector Direction;
  float Steering;
  if (!TargetLocations.empty())
  {
    Steering = GoToNextTargetLocation(Direction);
  }
  else
  {
    Steering = RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
  
    Direction = Vehicle->GetVehicleTransform().GetRotation().Rotator().Vector();
  }

  // Speed in km/h. cm/s ===> km/h
  const auto Speed = Vehicle->GetVehicleForwardSpeed() * 0.036f;
  if(Speed > 1.0f) {
      isOpenLog = true;
    } else {
      isOpenLog = false;
    }

  float Throttle;
  // if (TrafficLightState != ETrafficLightState::Green)
  // {
  //   Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::WaitingForRedLight);
  //   Throttle = Stop(Speed);
  // }
  // else if (IsThereAnObstacleAhead(*Vehicle, Speed, Direction))
  // {
  //   Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::ObstacleAhead);
  //   Throttle = Stop(Speed);
  // }
  // else
  // {
  //   Throttle = Move(Speed);
  // }
  Throttle = Move(Speed);

  if (Throttle < 0.001f)
  {
    // AutopilotControl.Brake = 1.0f;
    AutopilotControl.Brake = std::min(4 * fabs(Speed - SpeedLimit)/SpeedLimit, 1.0f);
    AutopilotControl.Throttle = 0.0f;
  }
  else
  {
    AutopilotControl.Brake = 0.0f;
    AutopilotControl.Throttle = Throttle;
  }
  AutopilotControl.Steer = Steering;

  last_AutopilotControl = AutopilotControl;
  return AutopilotControl;
}

float AWheeledVehicleAIController::GoToNextTargetLocation(FVector &Direction)
{
  // UE_LOG(LogCarla, Warning, TEXT("********* GoToNextTargetLocation() **********"));

  // Get middle point between the two front wheels.
  const auto CurrentLocation = [&]() {
    const auto &Wheels = Vehicle->GetVehicleMovementComponent()->Wheels;
    check((Wheels.Num() > 1) && (Wheels[0u] != nullptr) && (Wheels[1u] != nullptr));
    return (Wheels[0u]->Location + Wheels[1u]->Location) / 2.0f;
  } ();

  const auto Target = [&]() {
    const auto &Result = TargetLocations.front();

    return FVector{Result.X, Result.Y, CurrentLocation.Z};
  } ();
  if(isOpenLog) {
    UE_LOG(LogCarla, Warning, TEXT("GoToNextTargetLocation(), current location = (%f, %f, %f)"), CurrentLocation.X, CurrentLocation.Y, CurrentLocation.Z);
    UE_LOG(LogCarla, Warning, TEXT("GoToNextTargetLocation(), target location = (%f, %f, %f)"), Target.X, Target.Y, Target.Z);
    UE_LOG(LogCarla, Warning, TEXT("555, TargetLocations.size() = %d"), static_cast<int>(TargetLocations.size()));
  }
  
  if (Target.Equals(CurrentLocation, 150.0f))
  {
    if(TargetLocations.size() != speeds.size()) {
      UE_LOG(LogCarla, Error, TEXT("TargetLocations.size() %d != speeds.size() %d"),
             static_cast<int>(TargetLocations.size()), static_cast<int>(speeds.size()));
      ClearFixedRoute();
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
    if(TargetLocations.empty() && speeds.empty()) {
      UE_LOG(LogCarla, Error, TEXT("TargetLocations and speeds are empty"));
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
    TargetLocations.pop();
    speeds.pop();
    if(TargetLocations.empty() && speeds.empty()) {
      UE_LOG(LogCarla, Error, TEXT("TargetLocations and speeds are empty, SpeedLimit don't change"));
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
    SpeedLimit = speeds.front();
    UE_LOG(LogCarla, Warning, TEXT("------GoToNextTargetLocation(), new speed limit = %f km/h -----"), SpeedLimit);

    UE_LOG(LogCarla, Warning, TEXT("666, TargetLocations.size() = %d"), static_cast<int>(TargetLocations.size()));

    if (!TargetLocations.empty())
    {
      return GoToNextTargetLocation(Direction);
    }
    else
    {
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
  }

  Direction = (Target - CurrentLocation).GetSafeNormal();

  const FVector &Forward = GetPawn()->GetActorForwardVector();

  float dirAngle = Direction.UnitCartesianToSpherical().Y;
  float actorAngle = Forward.UnitCartesianToSpherical().Y;

  dirAngle *= (180.0f / PI);
  actorAngle *= (180.0 / PI);

  float angle = dirAngle - actorAngle;

  if (angle > 180.0f)
  {
    angle -= 360.0f;
  }
  else if (angle < -180.0f)
  {
    angle += 360.0f;
  }

  float Steering = 0.0f;
  if (angle < -MaximumSteerAngle)
  {
    Steering = -1.0f;
  }
  else if (angle > MaximumSteerAngle)
  {
    Steering = 1.0f;
  }
  else
  {
    Steering += angle / MaximumSteerAngle;
  }

  Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::FollowingFixedRoute);

  return Steering;
}

float AWheeledVehicleAIController::CalcStreeringValue(FVector &direction)
{
  UE_LOG(LogCarla, Warning, TEXT("********* CalcStreeringValue() **********"));

  float steering = 0;
  FVector BoxExtent = Vehicle->GetVehicleBoundingBoxExtent();
  FVector forward = Vehicle->GetActorForwardVector();

  FVector rightSensorPosition(BoxExtent.X / 2.0f, (BoxExtent.Y / 2.0f) + 100.0f, 0.0f);
  FVector leftSensorPosition(BoxExtent.X / 2.0f, -(BoxExtent.Y / 2.0f) - 100.0f, 0.0f);

  float forwardMagnitude = BoxExtent.X / 2.0f;

  float Magnitude =
      (float) sqrt(pow((double) leftSensorPosition.X, 2.0) + pow((double) leftSensorPosition.Y, 2.0));

  // same for the right and left
  float offset = FGenericPlatformMath::Acos(forwardMagnitude / Magnitude);

  float actorAngle = forward.UnitCartesianToSpherical().Y;

  float sinR = FGenericPlatformMath::Sin(actorAngle + offset);
  float cosR = FGenericPlatformMath::Cos(actorAngle + offset);

  float sinL = FGenericPlatformMath::Sin(actorAngle - offset);
  float cosL = FGenericPlatformMath::Cos(actorAngle - offset);

  rightSensorPosition.Y = sinR * Magnitude;
  rightSensorPosition.X = cosR * Magnitude;

  leftSensorPosition.Y = sinL * Magnitude;
  leftSensorPosition.X = cosL * Magnitude;

  FVector rightPositon = GetPawn()->GetActorLocation() + FVector(rightSensorPosition.X,
      rightSensorPosition.Y,
      0.0f);
  FVector leftPosition = GetPawn()->GetActorLocation() + FVector(leftSensorPosition.X,
      leftSensorPosition.Y,
      0.0f);

  FRoadMapPixelData rightRoadData = RoadMap->GetDataAt(rightPositon);
  if (!rightRoadData.IsRoad())
  {
    steering -= 0.2f;
  }

  FRoadMapPixelData leftRoadData = RoadMap->GetDataAt(leftPosition);
  if (!leftRoadData.IsRoad())
  {
    steering += 0.2f;
  }

  FRoadMapPixelData roadData = RoadMap->GetDataAt(GetPawn()->GetActorLocation());
  if (!roadData.IsRoad())
  {
    steering = 0.0f;
  }
  else if (roadData.HasDirection())
  {

    direction = roadData.GetDirection();
    FVector right = rightRoadData.GetDirection();
    FVector left = leftRoadData.GetDirection();

    forward.Z = 0.0f;

    float dirAngle = direction.UnitCartesianToSpherical().Y;
    float rightAngle = right.UnitCartesianToSpherical().Y;
    float leftAngle = left.UnitCartesianToSpherical().Y;

    dirAngle *= (180.0f / PI);
    rightAngle *= (180.0 / PI);
    leftAngle *= (180.0 / PI);
    actorAngle *= (180.0 / PI);

    float min = dirAngle - 90.0f;
    if (min < -180.0f)
    {
      min = 180.0f + (min + 180.0f);
    }

    float max = dirAngle + 90.0f;
    if (max > 180.0f)
    {
      max = -180.0f + (max - 180.0f);
    }

    if (dirAngle < -90.0 || dirAngle > 90.0)
    {
      if (rightAngle < min && rightAngle > max)
      {
        steering -= 0.2f;
      }
      if (leftAngle < min && leftAngle > max)
      {
        steering += 0.2f;
      }
    }
    else
    {
      if (rightAngle < min || rightAngle > max)
      {
        steering -= 0.2f;
      }
      if (leftAngle < min || leftAngle > max)
      {
        steering += 0.2f;
      }
    }

    float angle = dirAngle - actorAngle;

    if (angle > 180.0f)
    {
      angle -= 360.0f;
    }
    else if (angle < -180.0f)
    {
      angle += 360.0f;
    }

    if (angle < -MaximumSteerAngle)
    {
      steering = -1.0f;
    }
    else if (angle > MaximumSteerAngle)
    {
      steering = 1.0f;
    }
    else
    {
      steering += angle / MaximumSteerAngle;
    }
  }

  Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::FreeDriving);
 
  return steering;
}

float AWheeledVehicleAIController::Stop(const float Speed)
{
  UE_LOG(LogCarla, Warning, TEXT("********* Stop(), speed = %f km/h **********"), Speed);
  return (Speed >= 1.0f ? -Speed / SpeedLimit : 0.0f);
}

float AWheeledVehicleAIController::Move(const float Speed)
{
  if(isOpenLog) {
    UE_LOG(LogCarla, Warning, TEXT("********* Move(), speed = %f km/h, SpeedLimit = %f km/h **********"), Speed, SpeedLimit);
  }
  if (Speed > SpeedLimit)
  {
    // return Stop(Speed);
    return 0.0f;
  }
  // else if (Speed >= SpeedLimit - 10.0f)
  else if (Speed >= SpeedLimit * 0.95f)
  {
    return 0.3 + fabs(Speed - SpeedLimit)/SpeedLimit; 
  }
  else
  {
    return 1.0f;
  }
}
