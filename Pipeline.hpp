class Pipeline : public frc::VisionPipeline
{
    public:
        Pipeline(std::shared_ptr<nt::NetworkTable> networkTable);
        virtual void Process(cv::Mat& mat) override;

    private:
        std::shared_ptr<nt::NetworkTable>   _networkTable;
        int _value{ 0 };
};
