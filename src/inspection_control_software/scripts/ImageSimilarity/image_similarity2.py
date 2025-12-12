import argparse
import os
import sys

import torch
import torch.nn as nn
from PIL import Image, ImageOps
from torchvision import models, transforms


class ImgSimilarity:
    def __init__(self, model_name) -> None:
        self.model = self.load_and_modify_model(model_name)
        self.equalize = True

    def load_and_modify_model(self, model_name):
        """
        Loads a pretrained model and removes the last fully connected layer
        to create a feature extractor.
        """
        weights = "DEFAULT"  # Use best available weights
        model = None

        if model_name == "resnet50":
            model = models.resnet50(weights=weights)
            # ResNet final layer is named 'fc'
            input_features = model.fc.in_features
            model.fc = nn.Identity()

        elif model_name == "efficientnet_b0":
            model = models.efficientnet_b0(weights=weights)
            # EfficientNet final layer is inside 'classifier' sequence
            model.classifier = nn.Identity()

        elif model_name == "efficientnet_b1":
            model = models.efficientnet_b1(weights=weights)
            model.classifier = nn.Identity()

        elif model_name == "efficientnet_b4":
            model = models.efficientnet_b4(weights=weights)
            model.classifier = nn.Identity()

        elif model_name == "squeezenet":
            model = models.squeezenet1_1(weights=weights)
            # SqueezeNet is unique: it's Fully Convolutional.
            # The 'classifier' block contains the final conv and avgpool.
            # To get embeddings, we can remove the classifier and just use
            # Global Average Pooling on the features.
            model.classifier = nn.Sequential(nn.AdaptiveAvgPool2d((1, 1)), nn.Flatten())
        else:
            raise ValueError(f"Model {model_name} not supported.")

        model.eval()  # Set to evaluation mode
        return model

    def get_transforms(self, apply_equalize=False):
        """
        Returns the transformation pipeline.
        """
        transform_list = []

        if apply_equalize:
            # Custom transform for Histogram Equalization using PIL
            transform_list.append(transforms.Lambda(lambda img: ImageOps.equalize(img)))

        transform_list.extend(
            [
                transforms.Resize(256),
                transforms.CenterCrop(224),
                transforms.ToTensor(),
                # Standard ImageNet normalization
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
                ),
            ]
        )

        return transforms.Compose(transform_list)

    def process_image(self, image_path, transform, roi):
        """
        Loads and preprocesses a single image.
        """
        try:
            image = Image.open(image_path).convert("RGB")
            if roi:
                image = image.crop(roi)
                image.save("./reference_images/debug.jpg")
            return transform(image).unsqueeze(0)  # Add batch dimension
        except Exception as e:
            print(f"Error loading image {image_path}: {e}")
            return
            # sys.exit(1)

    def similarity(
        self,
        img1: tuple,
        img2: tuple,
        metric: str = "cosine",
    ) -> float:
        # 3. Prepare Transforms
        transform = self.get_transforms(apply_equalize=self.equalize)

        # 4. Process Images

        _img1, roi1 = img1
        _img2, roi2 = img2

        if not os.path.exists(_img1) or not os.path.exists(_img2):
            print("Error: One or both image paths are invalid.")
            return -100

        img1_tensor = self.process_image(_img1, transform, roi1)
        img2_tensor = self.process_image(_img2, transform, roi2)

        # 5. Extract Embeddings
        print("Extracting embeddings...")
        with torch.no_grad():
            embedding1 = self.model(img1_tensor)
            embedding2 = self.model(img2_tensor)

            # Flatten if necessary (though our modifications should handle this, safety check)
            embedding1 = torch.flatten(embedding1, start_dim=1)
            embedding2 = torch.flatten(embedding2, start_dim=1)

        # 6. Calculate Metric
        metric_label = ""
        score = -300

        if metric == "cosine":
            # Cosine Similarity: 1 is identical, -1 is opposite
            score = torch.nn.functional.cosine_similarity(embedding1, embedding2).item()
            metric_label = "Cosine Similarity"
        elif metric == "l1":
            # L1 (Manhattan) Distance: 0 is identical, higher is more different
            score = torch.nn.functional.pairwise_distance(
                embedding1, embedding2, p=1
            ).item()
            metric_label = "L1 Distance (Manhattan)"
        elif metric == "l2":
            # L2 (Euclidean) Distance: 0 is identical, higher is more different
            score = torch.nn.functional.pairwise_distance(
                embedding1, embedding2, p=2
            ).item()
            metric_label = "L2 Distance (Euclidean)"

        # print("\n" + "-" * 30)
        # print(f"Model: {args.model}")
        print(f"Histogram Equalization: {'Enabled' if self.equalize else 'Disabled'}")
        print(f"{metric_label}: {score:.4f}")
        return score


if __name__ == "__main__":
    main()
