# 如何操作：
在 platformio.ini 中，取消註解您想要編譯和上傳的那一行 src_filter。
像平常一樣，點擊 "Build" 和 "Upload"。PlatformIO 就只會編譯您指定的那一個 .cpp 檔案作為主程式。
當您想切換到另一個程式時，只需回到 platformio.ini，註解掉當前行，然後取消註解另一行即可。
