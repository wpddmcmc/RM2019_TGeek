
#include "task.h"
#include "zbar.h"
#include "detectQR.hpp"

bool task1(cv::Mat image, std::vector<int>& key)
{
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    cv::Mat imageGray;

    QR_mul::QR_detecter qr;
    if (qr.detectQR(image))
        qr.qr_thres.copyTo(imageGray);
    else
        cv::cvtColor(image, imageGray, CV_RGB2GRAY);
    
    int width = imageGray.cols;
    int height = imageGray.rows;
    uchar* raw = (uchar*)imageGray.data;
    zbar::Image imageZbar(width, height, "Y800", raw, width * height);
    scanner.scan(imageZbar);
    
    if (imageZbar.symbol_begin()==imageZbar.symbol_end())
    {
        std::cout << "scan QR is failed " << std::endl;

        imageZbar.set_data(NULL,0);
        return false;
    }
    else{
        for (zbar::Image::SymbolIterator symbol = imageZbar.symbol_begin(); 
             symbol != imageZbar.symbol_end(); ++symbol)
        {
            if(symbol->get_type_name() == "QR-Code")
            {
                std::cout << "Data: " << symbol->get_data() << std::endl;
                std::string result = symbol->get_data();
                // To-do 不够鲁棒
                for(int i = 0; i < 3; ++i)
                    key.push_back(static_cast<int>(result[i]-'0'));

                imageZbar.set_data(NULL,0);

                publish2car(key);

                return true;
            }
        }
    }

}
