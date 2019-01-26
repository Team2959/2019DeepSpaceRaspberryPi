class Pipeline : public frc::VisionPipeline
{
    public:
        Pipeline(std::shared_ptr<nt::NetworkTable> networkTable);
        virtual void Process(cv::Mat& mat) override;

    private:
        void FindCargo(cv::Mat& mat);
        void IncrementFrameNumber();

        std::shared_ptr<nt::NetworkTable>   m_networkTable;
        int                                 m_frameNumber{ 0 };
};
