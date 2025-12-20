#ifndef ZHTOPY_H
#define ZHTOPY_H

#include <vector>
#include <string>

/**
 * @brief Chinese to Pinyin converter for ESP-IDF
 * 
 * Usage:
 * ZhToPY::instance().zhToPY("中文");    // "zhongwen"
 * ZhToPY::instance().zhToJP("中文");    // "ZW"
 * ZhToPY::instance().zhToZM("中文");    // "Z"
 */
class ZhToPY {
public:
    // Get singleton instance (static, no dynamic allocation)
    static ZhToPY& instance() {
        static ZhToPY self;
        return self;
    }

    // Delete copy constructor and assignment
    ZhToPY(const ZhToPY&) = delete;
    ZhToPY& operator=(const ZhToPY&) = delete;

    // Convert Chinese to full pinyin
    std::string zhToPY(const std::string &chinese);
    
    // Convert Chinese to first letter abbreviation (简拼)
    std::string zhToJP(const std::string &chinese);
    
    // Convert Chinese to first character only (首字母)
    std::string zhToZM(const std::string &chinese);

private:
    ZhToPY();  // Private constructor
    
    // Static data stored in flash (const)
    static const char* pinyinArrStr;
    static const char* firstLetterStr;
    
    std::vector<std::string> listPY;
};

// Helper functions
std::vector<std::string> split(const std::string &s, char delimiter);
std::vector<std::string> split_name(const std::string &name);
int get_code_point(const std::string &name);

#endif // ZHTOPY_H