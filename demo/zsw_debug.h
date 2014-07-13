#ifndef ZSW_DEBUG_H
#define ZSW_DEBUG_H

#define ZSW_DEBUG_ON

#ifdef ZSW_DEBUG_ON

#define ZSW_DEBUG(msg) do{ cout << __FILE__ << ":" << __FUNCTION__ << ":" << __LINE__ << " error:" << msg << endl;  }while (0)
#endif

#endif /* ZSW_DEBUG_H */
