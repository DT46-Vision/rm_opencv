# 决策数据
std_msgs/Header header
uint16 self_sentry_hp     # 我方哨兵血量
uint16 self_hero_hp       # 我方英雄血量
uint16 self_infantry_hp   # 我方步兵血量
uint16 enemy_sentry_hp    # 敌方哨兵血量 
uint16 enemy_hero_hp      # 敌方英雄血量
uint16 enemy_infantry_hp  # 敌方步兵血量
uint16 remain_time        # 剩余时间 300s
uint16 remain_bullet      # 剩余子弹
uint8 match_progress      # 比赛进程 （准备为1,开始为4）
uint8 occupation          # 事件
uint8 color          # 颜色