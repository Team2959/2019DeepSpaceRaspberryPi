class Pipeline : public frc::VisionPipeline
{
    public:
        virtual void Process(cv::Mat& mat) override;

    private:
        int _value{ 0 };
};
