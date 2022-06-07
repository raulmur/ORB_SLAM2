/*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
*  All rights reserved.
*  Redistribution and use in source and binary forms, with or without modification, are permitted
* provided
*  that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this list of
* conditions and
*     the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice, this list of
* conditions and
*     the following disclaimer in the documentation and/or other materials provided with the
* distribution.
*
*  3. Neither the name of the copyright holder nor the names of its contributors may be used to
* endorse or
*     promote products derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY,
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE
*  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
 */

#pragma once

#include "mask_rcnn_classes.h"

enum class MaskRcnnClass : uint8_t
{
  Person = 0,
  Bicycle,
  Car,
  Motorcycle,
  Airplane,
  Bus,
  Train,
  Truck,
  Boat,
  TrafficLight,
  FireHydrant,
  StopSign,
  ParkingMeter,
  Bench,
  Bird,
  Cat,
  Dog,
  Horse,
  Sheep,
  Cow,
  Elephant,
  Bear,
  Zebra,
  Giraffe,
  Backpack,
  Umbrella,
  Handbag,
  Tie,
  Suitcase,
  Frisbee,
  Skis,
  Snowboard,
  SportsBall,
  Kite,
  BaseballBat,
  BaseballGlove,
  Skateboard,
  Surfboard,
  TennisRacket,
  Bottle,
  WineGlass,
  Cup,
  Fork,
  Knife,
  Spoon,
  Bowl,
  Banana,
  Apple,
  Sandwich,
  Orange,
  Broccoli,
  Carrot,
  HotDog,
  Pizza,
  Donut,
  Cake,
  Chair,
  Couch,
  PottedPlant,
  Bed,
  DiningTable,
  Toilet,
  Tv,
  Laptop,
  Mouse,
  Remote,
  Keyboard,
  CellPhone,
  Microwave,
  Oven,
  Toaster,
  Sink,
  Refrigerator,
  Book,
  Clock,
  Vase,
  Scissors,
  TeddyBear,
  HairDrier,
  Toothbrush
};

inline DnnObjectClass parseMaskRcnnClass(const MaskRcnnClass &mask_rcnn_class)
{

  std::function<uint8_t(MaskRcnnClass)> toUint8 = [](MaskRcnnClass mask_rcnn_class)
  {
    return static_cast<uint8_t>(mask_rcnn_class);
  };

  switch (mask_rcnn_class)
  {
  case MaskRcnnClass::Person:
    return {"Person",toUint8(MaskRcnnClass::Person)};
  case MaskRcnnClass::Bicycle:
    return {"Bicycle",toUint8(MaskRcnnClass::Bicycle)};
  case MaskRcnnClass::Car:
    return {"Car",toUint8(MaskRcnnClass::Car)};
  case MaskRcnnClass::Motorcycle:
    return {"Motorcycle",toUint8(MaskRcnnClass::Motorcycle)};
  case MaskRcnnClass::Airplane:
    return {"Airplane",toUint8(MaskRcnnClass::Airplane)};
  case MaskRcnnClass::Bus:
    return {"Bus",toUint8(MaskRcnnClass::Bus)};
  case MaskRcnnClass::Train:
    return {"Train",toUint8(MaskRcnnClass::Train)};
  case MaskRcnnClass::Truck:
    return {"Truck",toUint8(MaskRcnnClass::Truck)};
  case MaskRcnnClass::Boat:
    return {"Boat",toUint8(MaskRcnnClass::Boat)};
  case MaskRcnnClass::TrafficLight:
    return {"TrafficLight",toUint8(MaskRcnnClass::TrafficLight)};
  case MaskRcnnClass::FireHydrant:
    return {"FireHydrant",toUint8(MaskRcnnClass::FireHydrant)};
  case MaskRcnnClass::StopSign:
    return {"StopSign",toUint8(MaskRcnnClass::StopSign)};
  case MaskRcnnClass::ParkingMeter:
    return {"ParkingMeter",toUint8(MaskRcnnClass::ParkingMeter)};
  case MaskRcnnClass::Bench:
    return {"Bench",toUint8(MaskRcnnClass::Bench)};
  case MaskRcnnClass::Bird:
    return {"Bird",toUint8(MaskRcnnClass::Bird)};
  case MaskRcnnClass::Cat:
    return {"Cat",toUint8(MaskRcnnClass::Cat)};
  case MaskRcnnClass::Dog:
    return {"Dog",toUint8(MaskRcnnClass::Dog)};
  case MaskRcnnClass::Horse:
    return {"Horse",toUint8(MaskRcnnClass::Horse)};
  case MaskRcnnClass::Sheep:
    return {"Sheep",toUint8(MaskRcnnClass::Sheep)};
  case MaskRcnnClass::Cow:
    return {"Cow",toUint8(MaskRcnnClass::Cow)};
  case MaskRcnnClass::Elephant:
    return {"Elephant",toUint8(MaskRcnnClass::Elephant)};
  case MaskRcnnClass::Bear:
    return {"Bear",toUint8(MaskRcnnClass::Bear)};
  case MaskRcnnClass::Zebra:
    return {"Zebra",toUint8(MaskRcnnClass::Zebra)};
  case MaskRcnnClass::Giraffe:
    return {"Giraffe",toUint8(MaskRcnnClass::Giraffe)};
  case MaskRcnnClass::Backpack:
    return {"Backpack",toUint8(MaskRcnnClass::Backpack)};
  case MaskRcnnClass::Umbrella:
    return {"Umbrella",toUint8(MaskRcnnClass::Umbrella)};
  case MaskRcnnClass::Handbag:
    return {"Handbag",toUint8(MaskRcnnClass::Handbag)};
  case MaskRcnnClass::Tie:
    return {"Tie",toUint8(MaskRcnnClass::Tie)};
  case MaskRcnnClass::Suitcase:
    return {"Suitcase",toUint8(MaskRcnnClass::Suitcase)};
  case MaskRcnnClass::Frisbee:
    return {"Frisbee",toUint8(MaskRcnnClass::Frisbee)};
  case MaskRcnnClass::Skis:
    return {"Skis",toUint8(MaskRcnnClass::Skis)};
  case MaskRcnnClass::Snowboard:
    return {"Snowboard",toUint8(MaskRcnnClass::Snowboard)};
  case MaskRcnnClass::SportsBall:
    return {"SportsBall",toUint8(MaskRcnnClass::SportsBall)};
  case MaskRcnnClass::Kite:
    return {"Kite",toUint8(MaskRcnnClass::Kite)};
  case MaskRcnnClass::BaseballBat:
    return {"BaseballBat",toUint8(MaskRcnnClass::BaseballBat)};
  case MaskRcnnClass::BaseballGlove:
    return {"BaseballGlove",toUint8(MaskRcnnClass::BaseballGlove)};
  case MaskRcnnClass::Skateboard:
    return {"Skateboard",toUint8(MaskRcnnClass::Skateboard)};
  case MaskRcnnClass::Surfboard:
    return {"Surfboard",toUint8(MaskRcnnClass::Surfboard)};
  case MaskRcnnClass::TennisRacket:
    return {"TennisRacket",toUint8(MaskRcnnClass::TennisRacket)};
  case MaskRcnnClass::Bottle:
    return {"Bottle",toUint8(MaskRcnnClass::Bottle)};
  case MaskRcnnClass::WineGlass:
    return {"WineGlass",toUint8(MaskRcnnClass::WineGlass)};
  case MaskRcnnClass::Cup:
    return {"Cup",toUint8(MaskRcnnClass::Cup)};
  case MaskRcnnClass::Fork:
    return {"Fork",toUint8(MaskRcnnClass::Fork)};
  case MaskRcnnClass::Knife:
    return {"Knife",toUint8(MaskRcnnClass::Knife)};
  case MaskRcnnClass::Spoon:
    return {"Spoon",toUint8(MaskRcnnClass::Spoon)};
  case MaskRcnnClass::Bowl:
    return {"Bowl",toUint8(MaskRcnnClass::Bowl)};
  case MaskRcnnClass::Banana:
    return {"Banana",toUint8(MaskRcnnClass::Banana)};
  case MaskRcnnClass::Apple:
    return {"Apple",toUint8(MaskRcnnClass::Apple)};
  case MaskRcnnClass::Sandwich:
    return {"Sandwich",toUint8(MaskRcnnClass::Sandwich)};
  case MaskRcnnClass::Orange:
    return {"Orange",toUint8(MaskRcnnClass::Orange)};
  case MaskRcnnClass::Broccoli:
    return {"Broccoli",toUint8(MaskRcnnClass::Broccoli)};
  case MaskRcnnClass::Carrot:
    return {"Carrot",toUint8(MaskRcnnClass::Carrot)};
  case MaskRcnnClass::HotDog:
    return {"HotDog",toUint8(MaskRcnnClass::HotDog)};
  case MaskRcnnClass::Pizza:
    return {"Pizza",toUint8(MaskRcnnClass::Pizza)};
  case MaskRcnnClass::Donut:
    return {"Donut",toUint8(MaskRcnnClass::Donut)};
  case MaskRcnnClass::Cake:
    return {"Cake",toUint8(MaskRcnnClass::Cake)};
  case MaskRcnnClass::Chair:
    return {"Chair",toUint8(MaskRcnnClass::Chair)};
  case MaskRcnnClass::Couch:
    return {"Couch",toUint8(MaskRcnnClass::Couch)};
  case MaskRcnnClass::PottedPlant:
    return {"PottedPlant",toUint8(MaskRcnnClass::PottedPlant)};
  case MaskRcnnClass::Bed:
    return {"Bed",toUint8(MaskRcnnClass::Bed)};
  case MaskRcnnClass::DiningTable:
    return {"DiningTable",toUint8(MaskRcnnClass::DiningTable)};
  case MaskRcnnClass::Toilet:
    return {"Toilet",toUint8(MaskRcnnClass::Toilet)};
  case MaskRcnnClass::Tv:
    return {"Tv",toUint8(MaskRcnnClass::Tv)};
  case MaskRcnnClass::Laptop:
    return {"Laptop",toUint8(MaskRcnnClass::Laptop)};
  case MaskRcnnClass::Mouse:
    return {"Mouse",toUint8(MaskRcnnClass::Mouse)};
  case MaskRcnnClass::Remote:
    return {"Remote",toUint8(MaskRcnnClass::Remote)};
  case MaskRcnnClass::Keyboard:
    return {"Keyboard",toUint8(MaskRcnnClass::Keyboard)};
  case MaskRcnnClass::CellPhone:
    return {"CellPhone",toUint8(MaskRcnnClass::CellPhone)};
  case MaskRcnnClass::Microwave:
    return {"Microwave",toUint8(MaskRcnnClass::Microwave)};
  case MaskRcnnClass::Oven:
    return {"Oven",toUint8(MaskRcnnClass::Oven)};
  case MaskRcnnClass::Toaster:
    return {"Toaster",toUint8(MaskRcnnClass::Toaster)};
  case MaskRcnnClass::Sink:
    return {"Sink",toUint8(MaskRcnnClass::Sink)};
  case MaskRcnnClass::Refrigerator:
    return {"Refrigerator",toUint8(MaskRcnnClass::Refrigerator)};
  case MaskRcnnClass::Book:
    return {"Book",toUint8(MaskRcnnClass::Book)};
  case MaskRcnnClass::Clock:
    return {"Clock",toUint8(MaskRcnnClass::Clock)};
  case MaskRcnnClass::Vase:
    return {"Vase",toUint8(MaskRcnnClass::Vase)};
  case MaskRcnnClass::Scissors:
    return {"Scissors",toUint8(MaskRcnnClass::Scissors)};
  case MaskRcnnClass::TeddyBear:
    return {"TeddyBear",toUint8(MaskRcnnClass::TeddyBear)};
  case MaskRcnnClass::HairDrier:
    return {"HairDrier",toUint8(MaskRcnnClass::HairDrier)};
  case MaskRcnnClass::Toothbrush:
    return {"Toothbrush",toUint8(MaskRcnnClass::Toothbrush)};
  }

  throw std::invalid_argument("Invalid Mask Rcnn Class");
}