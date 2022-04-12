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
from crossing_v3 import Crossing

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

        # feature_dict_crossing1 = self.adjust_feature_dict(1, feature_dict)
        # self.crossing1 = Crossing(feature_dict_crossing1)

        feature_dict_crossing2 = self.adjust_feature_dict(2, feature_dict)
        self.crossing2 = Crossing(feature_dict_crossing2)

        # feature_dict_crossing3 = self.adjust_feature_dict(3, feature_dict)
        # self.crossing3 = Crossing(feature_dict_crossing3)

        # feature_dict_crossing4 = self.adjust_feature_dict(4, feature_dict)
        # self.crossing4 = Crossing(feature_dict_crossing4)
        
        # self.PCV = [round(i,3) for i in [sum(self.crossing1.PCV), sum(self.crossing2.PCV), sum(self.crossing3.PCV), sum(self.crossing4.PCV)]]
        # self.PSI_death = [round(i,3) for i in [self.crossing1.PSI_death, self.crossing2.PSI_death, self.crossing3.PSI_death, self.crossing4.PSI_death]]
        # self.PSI_injury = [round(i,3) for i in [self.crossing1.PSI_injury, self.crossing2.PSI_injury, self.crossing3.PSI_injury, self.crossing4.PSI_injury]]
    
    def df_to_dict(self, feature_df):
        feature_dict = {}
        for feature, value in zip(feature_df['feature'].values, feature_df['value'].values):
            feature_dict[feature] = value
        return feature_dict

    def adjust_feature_dict(self, cross_num, feature_dict):
        transform_table = {1:[0, 4,1,2,3],
                           2:[0, 1,2,3,4],
                           3:[0, 2,3,4,1],
                           4:[0, 3,4,1,2]}
        keys = [
            'width_a2', 'width_b2', 'width_c2', 'width_d2',
            'volume_P1', 'volume_P2', 'volume_P3', 'volume_P4',
            'volume_TH1', 'volume_TH2', 'volume_TH3', 'volume_TH4',
            'volume_RT1', 'volume_RT2', 'volume_RT3', 'volume_RT4',
            'volume_LT1', 'volume_LT2', 'volume_LT3', 'volume_LT4', 
            'postedSpeedLimit1', 'postedSpeedLimit2', 'postedSpeedLimit3', 'postedSpeedLimit4',
            'rightTurnRadius1', 'rightTurnRadius2', 'rightTurnRadius3', 'rightTurnRadius4',
            'leftTurnRadius1', 'leftTurnRadius2', 'leftTurnRadius3', 'leftTurnRadius4',
            'slipLane1', 'slipLane2', 'slipLane3','slipLane4',
            'shoulderType1', 'shoulderType2', 'shoulderType3', 'shoulderType4', # 0: shared, 1: RT only, 2:TH only (when there is slip lane)
            'RTOR1', 'RTOR2','RTOR3','RTOR4',
            'leftTurnType1', 'leftTurnType2', 'leftTurnType3', 'leftTurnType3',
            'laneNumber1', 'laneNumber2', 'laneNumber3', 'laneNumber4', 
            'leadingPedInterval1', 'leadingPedInterval2', 'leadingPedInterval3', 'leadingPedInterval4',
            'effectiveRed1', 'effectiveRed2', 'effectiveRed4', 'effectiveRed4', 
            'effectiveGreenPermissive1', 'effectiveGreenPermissive2', 'effectiveGreenPermissive3', 'effectiveGreenPermissive4',
            'walkInterval1', 'walkInterval2', 'walkInterval3', 'walkInterval4', 
            'flashingDontWalkInterval1', 'flashingDontWalkInterval2', 'flashingDontWalkInterval3', 'flashingDontWalkInterval4',
            'effectiveGreenProtectedLeftTurn1', 'effectiveGreenProtectedLeftTurn2', 'effectiveGreenProtectedLeftTurn3', 'effectiveGreenProtectedLeftTurn4',
            'effectiveGreenProtectedRightTurn1', 'effectiveGreenProtectedRightTurn2', 'effectiveGreenProtectedRightTurn3', 'effectiveGreenProtectedRightTurn4'
            ]
        feature_dict_adjusted = {}
        
        for key in keys:
            adjusted_key = key[:-1] + str(transform_table[cross_num][int(key[-1])])
            feature_dict_adjusted[key] = feature_dict[adjusted_key]
            # print('key: ', key, 'adj key: ', adjusted_key,'value: ', feature_dict[adjusted_key])

        keys = ['a_Frped', 'b_Frped',
                'baseSaturationFlow',
                'cycleTime',
                'pedWalkSpeed']
        for key in keys:
            feature_dict_adjusted[key] = feature_dict[key]
        
        return feature_dict_adjusted
    
        
