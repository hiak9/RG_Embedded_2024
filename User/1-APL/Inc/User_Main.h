/**
 * @file    User_Main.h
 * @brief   初始化，主循环函数C语言接口
 *
 * @author  Tang-yucheng (QQ: 3143961287)
 * @date    2024-2-14
 * @version v1.0
 */

#ifndef __APL_USER_MAIN_H
#define __APL_USER_MAIN_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/

/* 编译C语言接口 */
#ifdef __cplusplus
extern "C" {
#endif

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/
void User_setup(void);
void User_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* APL_User_Main.h */
