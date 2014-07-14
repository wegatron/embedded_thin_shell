#ifndef ZSW_DEBUG_H
#define ZSW_DEBUG_H

#define ZSW_DEBUG_ON

#ifdef ZSW_DEBUG_ON

#define ZSW_DEBUG(error_flag, msg) do{ if (error_flag) { cout << "[zsw_info]" <<  __FILE__ << ":" << __FUNCTION__ << ":" << __LINE__ << " error:" << msg << endl;}  }while (0)

#define ZSW_INFO(msg) do{cout << "[zsw_info]:" <<  msg << endl; }while (0)

#else
#define ZSW_DEBUG(error_flag, msg)
#deifne ZSW_INFO(msg)
#endif

#endif /* ZSW_DEBUG_H */
