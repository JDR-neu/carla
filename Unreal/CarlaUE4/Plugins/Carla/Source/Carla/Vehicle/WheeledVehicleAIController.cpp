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

// #include "opencv2/opencv.hpp"

// =============================================================================
// -- Static local methods -----------------------------------------------------
// =============================================================================

static bool RayCast(const AActor &Actor, const FVector &Start, const FVector &End)
{
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
  std::queue<T> EmptyQueue;
  Queue.swap(EmptyQueue);
}

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

AWheeledVehicleAIController::AWheeledVehicleAIController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));

  RandomEngine->Seed(RandomEngine->GenerateRandomSeed());

  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;

  isPlan = false;
  isOpenLog = false;
}

AWheeledVehicleAIController::~AWheeledVehicleAIController() {}

// =============================================================================
// -- AController --------------------------------------------------------------
// =============================================================================

void AWheeledVehicleAIController::OnPossess(APawn *aPawn)
{
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
}

void AWheeledVehicleAIController::OnUnPossess()
{
  Super::OnUnPossess();

  Vehicle = nullptr;
}

void AWheeledVehicleAIController::Tick(const float DeltaTime)
{
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
  bAutopilotEnabled = Enable;
  if (!KeepState)
  {
    // Reset state.
    Vehicle->SetSteeringInput(0.0f);
    Vehicle->SetThrottleInput(0.0f);
    Vehicle->SetBrakeInput(0.0f);
    Vehicle->SetReverse(false);
    Vehicle->SetHandbrakeInput(false);
    ClearQueue(TargetLocations);
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
  // UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRoute() **********"));
  isPlan = false;
  if (bOverwriteCurrent)
  {
    ClearQueue(TargetLocations);
  }
  for (auto &Location : Locations)
  {
    TargetLocations.emplace(Location);
  }
}

void AWheeledVehicleAIController::SetFixedRouteAll(const TArray<FVector> &locs) {
  UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRouteAll() **********"));
  isPlan = true;
  for(const auto& pt : locs) {
    TargetLocationsForPlan.emplace(pt);
  }
}


void AWheeledVehicleAIController::SetFixedRouteOnePoint(float x, float y, float z) {
  // UE_LOG(LogCarla, Warning, TEXT("********* SetFixedRouteOnePoint() **********"));
  isPlan = true;
  TargetLocationsForPlan.emplace(FVector(x, y, 0));
  SpeedLimitsForPlan.emplace(z);
  SpeedLimit = SpeedLimitsForPlan.front();

  // if(static_cast<int>(SpeedLimitsForPlan.size()) > 30 && static_cast<int>(TargetLocations.size()) < 50) {
  //   std::cout << "speed limits size in carla: " << static_cast<int>(SpeedLimitsForPlan.size()) << std::endl;
  //   auto temp_speeds = SpeedLimitsForPlan;
  //   for(int i=0; i<static_cast<int>(SpeedLimitsForPlan.size()); i++) {
  //     auto spd = temp_speeds.front();
  //     temp_speeds.pop();      
  //     std::cout << " " << spd;
  //   }
  //   std::cout << std::endl;
  // }
  /*
  if(static_cast<int>(SpeedLimitsForPlan.size()) > 30) {
    cv::Mat speeds_img(800, 420, CV_8UC3, cv::Scalar(255,255,255));
    cv::line(speeds_img, cv::Point(0, 720), cv::Point(400, 720), cv::Scalar(0,0,255), 2);
    cv::line(speeds_img, cv::Point(0, 500), cv::Point(400, 500), cv::Scalar(255,0,0), 2);
    cv::line(speeds_img, cv::Point(0, 300), cv::Point(400, 300), cv::Scalar(0,0,255), 2);
    for(int i=0; i<static_cast<int>(SpeedLimitsForPlan.size()); i++) {
      cv::circle(speeds_img, cv::Point(i*10, static_cast<int>(SpeedLimitsForPlan[i])), 2, cv::Scalar(0, 255, 0), -1);
    }
    cv::flip(speeds_img_, speeds_img_, 0);
    cv::imshow("Carla speed limit", speeds_img);
    cv::waitKey(1);
  }
  */
  /*
  const auto CurrentLocation = [&]() {
    const auto &Wheels = Vehicle->GetVehicleMovementComponent()->Wheels;
    check((Wheels.Num() > 1) && (Wheels[0u] != nullptr) && (Wheels[1u] != nullptr));
    return (Wheels[0u]->Location + Wheels[1u]->Location) / 2.0f;
  } ();
  std::cout << "CurrentLocation = (" << CurrentLocation.X << ", " << CurrentLocation.Y << ")"<< std::endl;
  std::cout << "SetRoutePoint   = (" << x << ", " << y << ")"<< std::endl;

  if(TargetLocations.size() > 35 && TargetLocations.size() < 60) {
    auto temp_route = TargetLocationsForPlan;
    int cnt = static_cast<int>(temp_route.size());
    auto temp_speeds = SpeedLimitsForPlan;
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
  ClearQueue(TargetLocationsForPlan);
  ClearQueue(SpeedLimitsForPlan);
}

// =============================================================================
// -- AI -----------------------------------------------------------------------
// =============================================================================

FVehicleControl AWheeledVehicleAIController::TickAutopilotController()
{
  if(isPlan) {
    return TickAutopilotControllerForPlan();
  }

#if WITH_EDITOR // This happens in simulation mode in editor.
  if (Vehicle == nullptr)
  {
    bAutopilotEnabled = false;
    return {};
  }
#endif // WITH_EDITOR

  check(Vehicle != nullptr);

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

  // Speed in km/h.
  const auto Speed = Vehicle->GetVehicleForwardSpeed() * 0.036f;

  float Throttle;
  // if (TrafficLightState != ETrafficLightState::Green)
  // {
  //   Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::WaitingForRedLight);
  //   Throttle = Stop(Speed);
  // }
  // else if (IsThereAnObstacleAhead(*Vehicle, Speed, Direction))
  if (IsThereAnObstacleAhead(*Vehicle, Speed, Direction))
  {
    Vehicle->SetAIVehicleState(ECarlaWheeledVehicleState::ObstacleAhead);
    Throttle = Stop(Speed);
  }
  else
  {
    Throttle = Move(Speed);
  }

  FVehicleControl AutopilotControl;

  if (Throttle < 0.001f)
  {
    AutopilotControl.Brake = 1.0f;
    AutopilotControl.Throttle = 0.0f;
  }
  else
  {
    AutopilotControl.Brake = 0.0f;
    AutopilotControl.Throttle = Throttle;
  }
  AutopilotControl.Steer = Steering;

  return AutopilotControl;
}

FVehicleControl AWheeledVehicleAIController::TickAutopilotControllerForPlan()
{
  check(Vehicle != nullptr);
  FVehicleControl AutopilotControl;

  if (TargetLocationsForPlan.empty())
  {
    if(isOpenLog) {
      UE_LOG(LogCarla, Warning, TEXT("TargetLocationsForPlan.size() = %d, path empty, car should stop **********"),
             static_cast<int>(TargetLocationsForPlan.size()));  
    }
    isOpenLog = false;
    AutopilotControl.Brake = 1.0f;
    AutopilotControl.Throttle = 0.0f;
    AutopilotControl.Steer = 0.0f;
    // return last_AutopilotControl;
    return AutopilotControl;
  }

  FVector Direction;
  float Steering;
  if (!TargetLocationsForPlan.empty())
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

  float Throttle = MoveForPlan(Speed);

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
  if(isPlan) {
    return GoToNextTargetLocationForPlan(Direction);
  }
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

  if (Target.Equals(CurrentLocation, 200.0f))
  {
    TargetLocations.pop();
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

float AWheeledVehicleAIController::GoToNextTargetLocationForPlan(FVector &Direction)
{
  // UE_LOG(LogCarla, Warning, TEXT("********* GoToNextTargetLocationForPlan() **********"));

  // Get middle point between the two front wheels.
  const auto CurrentLocation = [&]() {
    const auto &Wheels = Vehicle->GetVehicleMovementComponent()->Wheels;
    check((Wheels.Num() > 1) && (Wheels[0u] != nullptr) && (Wheels[1u] != nullptr));
    return (Wheels[0u]->Location + Wheels[1u]->Location) / 2.0f;
  } ();

  const auto Target = [&]() {
    const auto &Result = TargetLocationsForPlan.front();
    return FVector{Result.X, Result.Y, CurrentLocation.Z};
  } ();
  if(isOpenLog) {
    // UE_LOG(LogCarla, Warning, TEXT("GoToNextTargetLocationForPlan(), current location = (%f, %f, %f)"), CurrentLocation.X, CurrentLocation.Y, CurrentLocation.Z);
    // UE_LOG(LogCarla, Warning, TEXT("GoToNextTargetLocationForPlan(), target location = (%f, %f, %f)"), Target.X, Target.Y, Target.Z);
    // UE_LOG(LogCarla, Warning, TEXT("555, TargetLocationsForPlan.size() = %d"), static_cast<int>(TargetLocationsForPlan.size()));
  }
  
  if (Target.Equals(CurrentLocation, 150.0f))
  {
    if(TargetLocationsForPlan.size() != SpeedLimitsForPlan.size()) {
      UE_LOG(LogCarla, Error, TEXT("TargetLocationsForPlan.size() %d != SpeedLimitsForPlan.size() %d"),
             static_cast<int>(TargetLocationsForPlan.size()), static_cast<int>(SpeedLimitsForPlan.size()));
      ClearFixedRoute();
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
    if(TargetLocationsForPlan.empty() && SpeedLimitsForPlan.empty()) {
      UE_LOG(LogCarla, Error, TEXT("TargetLocationsForPlan and SpeedLimitsForPlan are empty"));
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
    TargetLocationsForPlan.pop();
    SpeedLimitsForPlan.pop();
    if(TargetLocationsForPlan.empty() && SpeedLimitsForPlan.empty()) {
      UE_LOG(LogCarla, Error, TEXT("TargetLocationsForPlan and SpeedLimitsForPlan are empty, SpeedLimit don't change"));
      return RoadMap != nullptr ? CalcStreeringValue(Direction) : 0.0f;
    }
    SpeedLimit = SpeedLimitsForPlan.front();
    UE_LOG(LogCarla, Warning, TEXT("------GoToNextTargetLocationForPlan(), new speed limit = %f km/h -----"), SpeedLimit);
    // UE_LOG(LogCarla, Warning, TEXT("666, TargetLocationsForPlan.size() = %d"), static_cast<int>(TargetLocationsForPlan.size()));

    if (!TargetLocationsForPlan.empty())
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
  return (Speed >= 1.0f ? -Speed / SpeedLimit : 0.0f);
}

float AWheeledVehicleAIController::Move(const float Speed)
{
  if (Speed >= SpeedLimit)
  {
    return Stop(Speed);
  }
  else if (Speed >= SpeedLimit - 10.0f)
  {
    return 0.5f;
  }
  else
  {
    return 1.0f;
  }
}

float AWheeledVehicleAIController::MoveForPlan(const float Speed)
{
  if(isOpenLog) {
    UE_LOG(LogCarla, Warning, TEXT("********* MoveForPlan(), speed = %f km/h, SpeedLimit = %f km/h **********"), Speed, SpeedLimit);
  }
  if (Speed > SpeedLimit)
  {    
    return 0.0f;
  }  
  else if (Speed >= SpeedLimit * 0.95f)
  {
    return 0.3 + fabs(Speed - SpeedLimit)/SpeedLimit; 
  }
  else
  {
    return 1.0f;
  }
}