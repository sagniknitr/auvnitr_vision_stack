// example matrix
Mat img = Mat::zeros(256,128, CV_32FC3);

// get the pointer (cast to data type of Mat)
float *pImgData = (float *)img.data;

// loop through rows, columns and channels
for (int row = 0; row < img.rows; ++row)
{
    for (int column = 0; column < img.cols; ++column)
    {
        for (int channel = 0; channel < img.channels(); ++channel)
        {
            float value = pImgData[img.channels() * (img.cols * row + col) + channel];
        }
    }
}
