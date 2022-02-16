#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Mohammad Zarei
# Created Date: 15 Feb 2022
# version = '1.0'
# ---------------------------------------------------------------------------
"""Implementation of an intersection class with four crossing for pedestrian risk index model"""
# ---------------------------------------------------------------------------
# Imports
from crossing import Crossing

class intersection:
    """represents a 4-legged intersection

    This intersection has four crossing
                

    Attributes:
        crossing1, crossing2, crossing3, crossing4: classes of each crossing
        PCV List[float]: Sum of potential conflicting volumes for each crossing - [PCV1, PCV2, PCV3, PCV4]
        PSI_death List[float]: Pedstrian safety index using death risk model for each crossing - [PSI_death1, PSI_death2, PSI_death3, PSI_death4]
        PSI_injury List[float]: Pedstrian safety index using severe injury risk model for each crossing - [PSI_injury1, PSI_injury2, PSI_injury3, PSI_injury4]
    """

    def __init__(self, feature_df):
        # Initializing crossings
        feature_dict = self.df_to_dict(feature_df)
        feature_dict_crossing1 = {
                        'width_a': feature_dict['width_a1'], 'width_b': feature_dict['width_b1'], 'width_c': feature_dict['width_c1'], 'width_d': feature_dict['width_d1'],
                        'volume_P1':feature_dict['volume_P4'], 'volume_P2': feature_dict['volume_P1'], 
                        'volume_TH1': feature_dict['volume_TH4'], 'volume_TH2': feature_dict['volume_TH1'], 'volume_TH4': feature_dict['volume_TH1'],
                        'volume_RT1': feature_dict['volume_RT4'], 'volume_RT2': feature_dict['volume_RT1'], 'volume_RT4': feature_dict['volume_RT3'],
                        'volume_LT3': feature_dict['volume_LT2'],
                        'postedSpeedLimit1': feature_dict['postedSpeedLimit4'], 'postedSpeedLimit2': feature_dict['postedSpeedLimit1'], 'postedSpeedLimit4': feature_dict['postedSpeedLimit3'],
                        'rightTurnRadius1': feature_dict['rightTurnRadius4'], 'rightTurnRadius2': feature_dict['rightTurnRadius1'], 'rightTurnRadius4': feature_dict['rightTurnRadius3'],
                        'leftTurnRadius3': feature_dict['leftTurnRadius2'],
                        'slipLane1': feature_dict['slipLane4'], 'slipLane2': feature_dict['slipLane1'], 'slipLane4': feature_dict['slipLane3'],
                        'shoulderType1': feature_dict['shoulderType4'], 'shoulderType2': feature_dict['shoulderType1'], 'shoulderType4': feature_dict['shoulderType3'], # 0: shared, 1: RT only, 2:TH only (when there is slip lane)
                        'RTOR1': feature_dict['RTOR4'], 'RTOR2': feature_dict['RTOR1'],
                        'leftTurnType3': feature_dict['leftTurnType2'],
                        'laneNumber1': feature_dict['laneNumber4'], 'laneNumber4': feature_dict['laneNumber3'],
                        'a_Frped': feature_dict['a_Frped'], 'b_Frped': feature_dict['b_Frped'],
                        'baseSaturationFlow': feature_dict['baseSaturationFlow'],
                        'cycleTime': feature_dict['cycleTime'],
                        'leadingPedInterval1':feature_dict['leadingPedInterval4'], 'leadingPedInterval2':feature_dict['leadingPedInterval1'],
                        'effectiveRed1': feature_dict['effectiveRed4'], 'effectiveRed2': feature_dict['effectiveRed1'], 
                        'effectiveGreen1': feature_dict['effectiveGreen4'], 'effectiveGreen2': feature_dict['effectiveGreen1'],
                        'walkInterval1': feature_dict['walkInterval4'], 'walkInterval2': feature_dict['walkInterval1'], 
                        'flashingDontWalkInterval1': feature_dict['flashingDontWalkInterval4'], 'flashingDontWalkInterval2': feature_dict['flashingDontWalkInterval1'],
                        'effectiveGreenProtected1': feature_dict['effectiveGreenProtected4'],
                        'pedWalkSpeed': feature_dict['pedWalkSpeed'],
                        }
        self.crossing1 = Crossing(feature_dict_crossing1)
        feature_dict_crossing2 = {
                        'width_a': feature_dict['width_a2'], 'width_b': feature_dict['width_b2'], 'width_c': feature_dict['width_c2'], 'width_d': feature_dict['width_d2'],
                        'volume_P1':feature_dict['volume_P1'], 'volume_P2': feature_dict['volume_P2'], 
                        'volume_TH1': feature_dict['volume_TH1'], 'volume_TH2': feature_dict['volume_TH2'], 'volume_TH4': feature_dict['volume_TH4'],
                        'volume_RT1': feature_dict['volume_RT1'], 'volume_RT2': feature_dict['volume_RT2'], 'volume_RT4': feature_dict['volume_RT4'],
                        'volume_LT3': feature_dict['volume_LT3'],
                        'postedSpeedLimit1': feature_dict['postedSpeedLimit1'], 'postedSpeedLimit2': feature_dict['postedSpeedLimit2'], 'postedSpeedLimit4': feature_dict['postedSpeedLimit4'],
                        'rightTurnRadius1': feature_dict['rightTurnRadius1'], 'rightTurnRadius2': feature_dict['rightTurnRadius2'], 'rightTurnRadius4': feature_dict['rightTurnRadius4'],
                        'leftTurnRadius3': feature_dict['leftTurnRadius3'],
                        'slipLane1': feature_dict['slipLane1'], 'slipLane2': feature_dict['slipLane2'], 'slipLane4': feature_dict['slipLane4'],
                        'shoulderType1': feature_dict['shoulderType1'], 'shoulderType2': feature_dict['shoulderType2'], 'shoulderType4': feature_dict['shoulderType4'], # 0: shared, 1: RT only, 2:TH only (when there is slip lane)
                        'RTOR1': feature_dict['RTOR1'], 'RTOR2': feature_dict['RTOR2'],
                        'leftTurnType3': feature_dict['leftTurnType3'],
                        'laneNumber1': feature_dict['laneNumber1'], 'laneNumber4': feature_dict['laneNumber4'],
                        'a_Frped': feature_dict['a_Frped'], 'b_Frped': feature_dict['b_Frped'],
                        'baseSaturationFlow': feature_dict['baseSaturationFlow'],
                        'cycleTime': feature_dict['cycleTime'],
                        'leadingPedInterval1':feature_dict['leadingPedInterval1'], 'leadingPedInterval2':feature_dict['leadingPedInterval2'],
                        'effectiveRed1': feature_dict['effectiveRed1'], 'effectiveRed2': feature_dict['effectiveRed2'], 
                        'effectiveGreen1': feature_dict['effectiveGreen1'], 'effectiveGreen2': feature_dict['effectiveGreen2'],
                        'walkInterval1': feature_dict['walkInterval1'], 'walkInterval2': feature_dict['walkInterval2'], 
                        'flashingDontWalkInterval1': feature_dict['flashingDontWalkInterval1'], 'flashingDontWalkInterval2': feature_dict['flashingDontWalkInterval2'],
                        'effectiveGreenProtected1': feature_dict['effectiveGreenProtected1'],
                        'pedWalkSpeed': feature_dict['pedWalkSpeed'],
                        }
        self.crossing2 = Crossing(feature_dict_crossing2)
        feature_dict_crossing3 = {
                        'width_a': feature_dict['width_a3'], 'width_b': feature_dict['width_b3'], 'width_c': feature_dict['width_c3'], 'width_d': feature_dict['width_d3'],
                        'volume_P1':feature_dict['volume_P2'], 'volume_P2': feature_dict['volume_P3'], 
                        'volume_TH1': feature_dict['volume_TH2'], 'volume_TH2': feature_dict['volume_TH3'], 'volume_TH4': feature_dict['volume_TH1'],
                        'volume_RT1': feature_dict['volume_RT2'], 'volume_RT2': feature_dict['volume_RT3'], 'volume_RT4': feature_dict['volume_RT1'],
                        'volume_LT3': feature_dict['volume_LT4'],
                        'postedSpeedLimit1': feature_dict['postedSpeedLimit2'], 'postedSpeedLimit2': feature_dict['postedSpeedLimit3'], 'postedSpeedLimit4': feature_dict['postedSpeedLimit1'],
                        'rightTurnRadius1': feature_dict['rightTurnRadius2'], 'rightTurnRadius2': feature_dict['rightTurnRadius3'], 'rightTurnRadius4': feature_dict['rightTurnRadius1'],
                        'leftTurnRadius3': feature_dict['leftTurnRadius4'],
                        'slipLane1': feature_dict['slipLane2'], 'slipLane2': feature_dict['slipLane3'], 'slipLane4': feature_dict['slipLane1'],
                        'shoulderType1': feature_dict['shoulderType2'], 'shoulderType2': feature_dict['shoulderType3'], 'shoulderType4': feature_dict['shoulderType1'], # 0: shared, 1: RT only, 2:TH only (when there is slip lane)
                        'RTOR1': feature_dict['RTOR2'], 'RTOR2': feature_dict['RTOR3'],
                        'leftTurnType3': feature_dict['leftTurnType4'],
                        'laneNumber1': feature_dict['laneNumber2'], 'laneNumber4': feature_dict['laneNumber1'],
                        'a_Frped': feature_dict['a_Frped'], 'b_Frped': feature_dict['b_Frped'],
                        'baseSaturationFlow': feature_dict['baseSaturationFlow'],
                        'cycleTime': feature_dict['cycleTime'],
                        'leadingPedInterval1':feature_dict['leadingPedInterval2'], 'leadingPedInterval2':feature_dict['leadingPedInterval3'],
                        'effectiveRed1': feature_dict['effectiveRed2'], 'effectiveRed2': feature_dict['effectiveRed3'], 
                        'effectiveGreen1': feature_dict['effectiveGreen2'], 'effectiveGreen2': feature_dict['effectiveGreen3'],
                        'walkInterval1': feature_dict['walkInterval2'], 'walkInterval2': feature_dict['walkInterval3'], 
                        'flashingDontWalkInterval1': feature_dict['flashingDontWalkInterval2'], 'flashingDontWalkInterval2': feature_dict['flashingDontWalkInterval3'],
                        'effectiveGreenProtected1': feature_dict['effectiveGreenProtected2'],
                        'pedWalkSpeed': feature_dict['pedWalkSpeed'],
                        }
        self.crossing3 = Crossing(feature_dict_crossing3)
        feature_dict_crossing4 = {
                        'width_a': feature_dict['width_a4'], 'width_b': feature_dict['width_b4'], 'width_c': feature_dict['width_c4'], 'width_d': feature_dict['width_d4'],
                        'volume_P1':feature_dict['volume_P2'], 'volume_P2': feature_dict['volume_P3'], 
                        'volume_TH1': feature_dict['volume_TH3'], 'volume_TH2': feature_dict['volume_TH4'], 'volume_TH4': feature_dict['volume_TH2'],
                        'volume_RT1': feature_dict['volume_RT3'], 'volume_RT2': feature_dict['volume_RT4'], 'volume_RT4': feature_dict['volume_RT2'],
                        'volume_LT3': feature_dict['volume_LT1'],
                        'postedSpeedLimit1': feature_dict['postedSpeedLimit3'], 'postedSpeedLimit2': feature_dict['postedSpeedLimit4'], 'postedSpeedLimit4': feature_dict['postedSpeedLimit2'],
                        'rightTurnRadius1': feature_dict['rightTurnRadius3'], 'rightTurnRadius2': feature_dict['rightTurnRadius4'], 'rightTurnRadius4': feature_dict['rightTurnRadius2'],
                        'leftTurnRadius3': feature_dict['leftTurnRadius1'],
                        'slipLane1': feature_dict['slipLane3'], 'slipLane2': feature_dict['slipLane4'], 'slipLane4': feature_dict['slipLane2'],
                        'shoulderType1': feature_dict['shoulderType3'], 'shoulderType2': feature_dict['shoulderType4'], 'shoulderType4': feature_dict['shoulderType2'], # 0: shared, 1: RT only, 2:TH only (when there is slip lane)
                        'RTOR1': feature_dict['RTOR3'], 'RTOR2': feature_dict['RTOR4'],
                        'leftTurnType3': feature_dict['leftTurnType1'],
                        'laneNumber1': feature_dict['laneNumber3'], 'laneNumber4': feature_dict['laneNumber2'],
                        'a_Frped': feature_dict['a_Frped'], 'b_Frped': feature_dict['b_Frped'],
                        'baseSaturationFlow': feature_dict['baseSaturationFlow'],
                        'cycleTime': feature_dict['cycleTime'],
                        'leadingPedInterval1':feature_dict['leadingPedInterval3'], 'leadingPedInterval2':feature_dict['leadingPedInterval4'],
                        'effectiveRed1': feature_dict['effectiveRed3'], 'effectiveRed2': feature_dict['effectiveRed4'], 
                        'effectiveGreen1': feature_dict['effectiveGreen3'], 'effectiveGreen2': feature_dict['effectiveGreen4'],
                        'walkInterval1': feature_dict['walkInterval2'], 'walkInterval3': feature_dict['walkInterval4'], 
                        'flashingDontWalkInterval1': feature_dict['flashingDontWalkInterval3'], 'flashingDontWalkInterval2': feature_dict['flashingDontWalkInterval4'],
                        'effectiveGreenProtected1': feature_dict['effectiveGreenProtected3'],
                        'pedWalkSpeed': feature_dict['pedWalkSpeed'],
                        }
        self.crossing4 = Crossing(feature_dict_crossing4)
        
        self.PCV = [sum(self.crossing1.PCV), sum(self.crossing2.PCV), sum(self.crossing3.PCV), sum(self.crossing4.PCV)]
        self.PSI_death = [self.crossing1.PSI_death, self.crossing2.PSI_death, self.crossing3.PSI_death, self.crossing4.PSI_death]
        self.PSI_injury = [self.crossing1.PSI_injury, self.crossing2.PSI_injury, self.crossing3.PSI_injury, self.crossing4.PSI_injury]
    
    def df_to_dict(self, feature_df):
        feature_dict = {}
        for feature, value in zip(feature_df['feature'].values, feature_df['value'].values):
            feature_dict[feature] = value
        return feature_dict

    
        
