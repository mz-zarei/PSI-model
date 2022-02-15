#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 10 Feb 2022
# version = '0.1'
# ---------------------------------------------------------------------------
"""Test file for crossing class implementation"""
# ---------------------------------------------------------------------------
# Imports
from crossing import Crossing

feature_dict_crossing = {'width_a': 12.9, 'width_b': 13.12, 'width_c': 3, 'width_d': 3,
                        'volume_P1':100, 'volume_P2': 75, 
                        'volume_TH1': 386, 'volume_TH2': 498, 'volume_TH4': 714,
                        'volume_RT1': 374, 'volume_RT2': 211, 'volume_RT4': 315,
                        'volume_LT3': 156,
                        'postedSpeedLimit1': 50, 'postedSpeedLimit2': 50, 'postedSpeedLimit4': 50,
                        'rightTurnRadius1': 9.34, 'rightTurnRadius2': 12.71, 'rightTurnRadius4': 11.82,
                        'leftTurnRadius3': 19,
                        'slipLane1': False, 'slipLane2': False, 'slipLane4': False,
                        'shoulderType1': 1, 'shoulderType2': 1, 'shoulderType4': 1, # 0: shared, 1: RT only, 2:TH only (when there is slip lane)
                        'RTOR1': True, 'RTOR2': True,
                        'leftTurnType3': "permissive",
                        'laneNumber1': 2, 'laneNumber4': 3,
                        'a_Frped': 0.49, 'b_Frped': 10645,
                        'baseSaturationFlow': 1800,
                        'cycleTime': 94,
                        'leadingPedInterval1':0, 'leadingPedInterval2':0,
                        'effectiveRed1': 57, 'effectiveRed2': 69, 
                        'effectiveGreen1': 37, 'effectiveGreen2': 25,
                        'walkInterval1': 26, 'walkInterval2': 14, 
                        'flashingDontWalkInterval1': 10, 'flashingDontWalkInterval2': 10,
                        'effectiveGreenProtected1': 6,
                        'pedWalkSpeed': 1,

                        }

crossing1 = Crossing(feature_dict_crossing)
print(crossing1.PCV)
print(crossing1.PPP)
print(crossing1.CS)
print(crossing1.DR)
print(crossing1.SIR)


