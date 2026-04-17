"""
Filename: l5.py

Lab 5 (Python file version): Classification using Decision Trees
Dataset: Titanic Survival

Instructions:
1. Complete all "to do" blocks marked in this file.
2. Do not change function names or function signatures.
3. Run this file after each section to verify your progress.

Suggested run command:
    python l5.py
"""
from __future__ import annotations
from pathlib import Path
import pickle

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier, plot_tree

np.random.seed(1)

# Fixed column names from Titanic.csv
FEATURE_COLUMNS = [
    "pclass",
    "gender",
    "age",
    "sibsp",
    "parch",
    "fare",
    "embarked_C",
    "embarked_Q",
    "embarked_S",
]
TARGET_COLUMN = "survived"



# Q1. Reading the Data
def load_titanic_data(data_path: str) -> pd.DataFrame:
    """
    Read the Titanic dataset from CSV and return a DataFrame.

    Args:
        data_path: Path to Titanic.csv

    Returns:
        DataFrame with 10 columns (9 features + 1 label).
    """
    # TODO: read the csv file using the header already present in Titanic.csv
    df = pd.read_csv("Titanic.csv")
    print(f"Dataset loaded with shape: {df.shape}, Dataset head:")
    print(df.head())
    print()
    return df


def preprocess_data(df: pd.DataFrame) -> tuple[pd.DataFrame, pd.Series]:
    """
    Prepare Titanic features and labels for modeling.

    Note:
    - The dataset is already numeric.
    - `gender` is already binary encoded.
    - `embarked_*` columns are already one-hot encoded.

    Args:
        df: Raw DataFrame from load_titanic_data

    Returns:
        X: feature DataFrame containing FEATURE_COLUMNS
        y: label Series containing TARGET_COLUMN
    """
    # TODO:
    # 1) Copy the input DataFrame
    # 2) Select FEATURE_COLUMNS into X
    # 3) Select TARGET_COLUMN into y
    # 4) Return X and y
    X = df.drop(columns=['survived'])
    y = df['survived']
    print(f"X loaded with shape: {X.shape}, Dataset head:")
    print(X.head())
    print(f"y loaded with shape: {y.shape}, Dataset head:")
    print(y.head())
    print()
    return X, y




#Q2. Creating the Train-Test Split
def create_train_test_split(
    X: pd.DataFrame,
    y: pd.Series,
    test_size: float = 0.2,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.Series, pd.Series]:
    """
    Split dataset into train and test partitions.

    Use stratified split to preserve class ratio.
    """
    # TODO: call train_test_split with stratify=y
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=test_size, stratify=y, random_state=1)
    print(f"X_train loaded with shape: {X_train.shape}, Dataset head:")
    print(X_train.head())
    print(f"X_test loaded with shape: {X_test.shape}, Dataset head:")
    print(X_test.head())
    print(f"y_train loaded with shape: {y_train.shape}, Dataset head:")
    print(y_train.head())
    print(f"y_test loaded with shape: {y_test.shape}, Dataset head:")
    print(y_test.head())
    print()
    return X_train, X_test, y_train, y_test



#Q3 and Q4 Train Decision Tree (for baseline and constrained DT)
def train_decision_tree(
    X_train: pd.DataFrame,
    y_train: pd.Series,
    max_depth: int | None = None,
) -> DecisionTreeClassifier:
    """
    Train a DecisionTreeClassifier and return the fitted model.

    For baseline model, keep max_depth=None.
    For constrained model, set max_depth=3.
    """
    # TODO:
    # 1) Initialize DecisionTreeClassifier from sklearn with the given max_depth
    # 2) Fit on X_train, y_train
    # 3) Return fitted model
    dtc = DecisionTreeClassifier(max_depth=max_depth)
    dtc.fit(X_train, y_train)
    return dtc


def plot_decision_tree(
    model: DecisionTreeClassifier,
    feature_names: list[str],
    class_names: list[str] | None = None,
    save_path: str | Path = "plots/tree_depth3.png",
) -> None:
    """
    Visualize a trained DecisionTreeClassifier using plot_tree.

    Args:
        model: A fitted DecisionTreeClassifier
        feature_names: List of feature column names for node labels
        class_names: Optional list of class label names (e.g. ["Not Survived", "Survived"])
        save_path: Path to save the plot image
    """
    # TODO:
    # 1) Create a matplotlib figure with an appropriate size (e.g. figsize=(20, 10))
    # 2) Call plot_tree with the model, feature_names, class_names, and filled=True
    # 3) Add a title to the plot (e.g. "Decision Tree Visualization")
    # 4) Save the plot to save_path and close the figure
    plt.figure(figsize=(20, 10))
    plot_tree(model, feature_names=feature_names, class_names=class_names, filled=True)
    plt.title("Decision Tree Visualization")
    plt.savefig(save_path)
    plt.close()


def top_k_features(
    model: DecisionTreeClassifier,
    feature_names: list[str],
    k: int = 2,
) -> list[tuple[str, float]]:
    """
    Return top-k important features as (feature_name, importance).

    Deterministic tie-break:
    - Sort by importance descending
    - If equal, sort by feature name ascending
    """
    # TODO: implement top_k_features with the deterministic tie-break
    # Hint: use model.feature_importances_
    sorted_importances = sorted(zip(feature_names, model.feature_importances_), key=lambda x: (-x[1], x[0]))
    return sorted_importances[:k]


def evaluate_model(
    model: DecisionTreeClassifier,
    X_train: pd.DataFrame,
    y_train: pd.Series,
    X_test: pd.DataFrame,
    y_test: pd.Series,
    feature_names: list[str],
) -> dict:
    """
    Evaluate model and return summary dictionary.

    Required dictionary keys:
        train_accuracy, test_accuracy, depth, top_2_features
    """
    # TODO: compute train/test accuracy, tree depth, and top 2 features using top_k_features
    summary = {}
    summary["train_accuracy"] = model.score(X_train, y_train)
    summary["test_accuracy"] = model.score(X_test, y_test)
    summary["depth"] = model.get_depth()
    summary["top_2_features"] = top_k_features(model, X_train.columns, k=2)
    print(f"Model params: {model.get_params()}")
    print(f"Train Accuracy: {summary['train_accuracy']}")
    print(f"Test Accuracy: {summary['test_accuracy']}")
    print(f"Feature importances: {model.feature_importances_}")
    print(f"Top 2 features: {summary['top_2_features']}")
    print(f"Depth: {summary['depth']}")
    print()
    return summary



#Q5 Add noise to the training dataset
def flip_label(lab: int, p_noise: float, rng: np.random.Generator) -> int:
    """
    Flip binary label with probability p_noise.

    If random_value < p_noise, convert 0->1 or 1->0.
    """
    # TODO: implement probabilistic label flip using rng
    val = rng.random()
    if val < p_noise:
        return 1-lab
    else:
        return lab


def add_label_noise(
    y_train: pd.Series,
    p_noise: float = 0.1,
    seed: int = 42,
) -> pd.Series:
    """
    Create noisy labels by flipping each training label with probability p_noise.

    Args:
        y_train: clean training labels (0/1)
        p_noise: probability of flipping each label
        seed: random seed for reproducibility

    Returns:
        Noisy labels Series with same index as y_train.
    """
    # TODO:
    # 1) Create rng = np.random.default_rng(seed)
    # 2) Apply flip_label over y_train
    # 3) Return noisy label Series
    rng = np.random.default_rng(seed)
    flip_labels = y_train.apply(lambda lab: flip_label(lab, p_noise, rng))
    return flip_labels

#Q6 and Q7. Train a DT on the noisy dataset
def run_noise_experiment(
    X_train: pd.DataFrame,
    y_train: pd.Series,
    X_test: pd.DataFrame,
    y_test: pd.Series,
    feature_names: list[str],
    p_noise: float = 0.1,
    seed: int = 42,
) -> dict:
    """
    Train/evaluate a decision tree on noisy labels and return summary.

    Required dictionary keys:
        train_accuracy, test_accuracy, depth, top_2_features, noisy_labels
    """
    # TODO:
    # 1) Create noisy labels with add_label_noise
    # 2) Train default tree on (X_train, noisy_labels)
    # 3) Evaluate using evaluate_model
    # 4) Include noisy_labels in returned dictionary
    raise NotImplementedError("to do: Implement run_noise_experiment")

# #Q7. Repeating noise experiments for observations
# def repeat_noise_experiments(
#     X_train: pd.DataFrame,
#     y_train: pd.Series,
#     X_test: pd.DataFrame,
#     y_test: pd.Series,
#     feature_names: list[str],
#     p_noise: float = 0.1,
#     n_runs: int = 5,
#     base_seed: int = 100,
# ) -> dict:
#     """
#     Repeat noise experiment n_runs times and return aggregate statistics.

#     Returns:
#     ((average_train_accuracy, average_test_accuracy, average_depth, top_2_feature_frequencies))
#     """
#     raise NotImplementedError("to do: Implement repeat_noise_experiments")
    


#Q8 and Q9. Train and test accuracy plot vs depth for observations
def depth_accuracy_curve(
    X_train: pd.DataFrame,
    y_train: pd.Series,
    X_test: pd.DataFrame,
    y_test: pd.Series,
    depth_values: list[int],
) -> tuple[list[int], list[float], list[float]]:
    """
    Compute train/test accuracy for each depth in depth_values.

    Returns:
        (depth_values, train_accuracies, test_accuracies)
    """
    # TODO: train one tree per depth and collect score lists
    raise NotImplementedError("to do: Implement depth_accuracy_curve")


def plot_depth_accuracy(
    depth_values: list[int],
    train_accuracies: list[float],
    test_accuracies: list[float],
) -> None:
    """
    Plot train and test accuracy curves against tree depth.
    """
    plt.figure(figsize=(10, 5))
    plt.plot(depth_values, train_accuracies, marker="o", label="Train Accuracy")
    plt.plot(depth_values, test_accuracies, marker="s", label="Test Accuracy")
    plt.xlabel("max_depth")
    plt.ylabel("Accuracy")
    plt.title("Decision Tree Accuracy vs max_depth")
    plt.xticks(depth_values)
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()


def save_model(model: DecisionTreeClassifier, model_path: str | Path) -> None:
    """
    Save a trained model to a pickle file.
    """
    model_path = Path(model_path)
    model_path.parent.mkdir(parents=True, exist_ok=True)
    with model_path.open("wb") as file:
        pickle.dump(model, file)


def load_model(model_path: str | Path) -> DecisionTreeClassifier:
    """
    Load a trained model from a pickle file.
    """
    model_path = Path(model_path)
    with model_path.open("rb") as file:
        return pickle.load(file)



def print_summary_block(title: str, summary: dict) -> None:
    """
    Helper to print metric blocks in a consistent format.
    """
    print(f"\n{'=' * 60}")
    print(title)
    print("=" * 60)
    for key, value in summary.items():
        if key not in {"model", "noisy_labels"}:
            print(f"{key}: {value}")


def main() -> None:
    """
    End-to-end runner for the assignment.

    This lets students run the file and view outputs section-wise.
    """
    dataset_path = Path(__file__).parent / "Titanic.csv"
    model_dir = Path(__file__).parent / "saved_models"
    plot_dir = Path(__file__).parent / "plots"

    try:
        model_dir.mkdir(parents=True, exist_ok=True)
        plot_dir.mkdir(parents=True, exist_ok=True)

        df = load_titanic_data(str(dataset_path))
        X, y = preprocess_data(df)

        X_train, X_test, y_train, y_test = create_train_test_split(X, y)

        # Q3: Train and evaluate baseline decision tree
        baseline_model = train_decision_tree(X_train, y_train, max_depth=None)
        save_model(baseline_model, model_dir / "decision_tree_baseline.pkl")
        baseline_summary = evaluate_model(
            baseline_model,
            X_train,
            y_train,
            X_test,
            y_test,
            feature_names=list(X.columns),
        )
        print_summary_block("Baseline Decision Tree", baseline_summary)

        # Q4: Train and evaluate depth-constrained tree
        depth3_model = train_decision_tree(X_train, y_train, max_depth=3)
        save_model(depth3_model, model_dir / "decision_tree_depth3.pkl")
        depth3_summary = evaluate_model(
            depth3_model,
            X_train,
            y_train,
            X_test,
            y_test,
            feature_names=list(X.columns),
        )
        print_summary_block("Decision Tree with max_depth=3", depth3_summary)

        # Save Q4 tree plot
        plot_decision_tree(
            depth3_model,
            feature_names=list(X.columns),
            class_names=["Not Survived", "Survived"],
            save_path=plot_dir / "tree_depth3.png",
        )

        # Q6: Train and evaluate one noisy model
        noisy_summary = run_noise_experiment(
            X_train,
            y_train,
            X_test,
            y_test,
            feature_names=list(X.columns),
            p_noise=0.1,
            seed=1,
        )
        save_model(noisy_summary["model"], model_dir / "clf_noisy.pkl")

        noisy_summary_to_print = {
            k: v for k, v in noisy_summary.items()
            if k not in {"model", "noisy_labels"}
        }
        print_summary_block("Decision Tree on Noisy Labels", noisy_summary_to_print)

        # Q7: Print per-seed repeated noise experiments for seeds 0 to 4
        print("\n" + "=" * 80)
        print("Q7 Repeated Noise Experiments")
        print("=" * 80)
        print("Seed | Train Accuracy | Test Accuracy | Depth | Top Feature 1 | Top Feature 2")
        print("-" * 80)

        for seed in range(5):
            summary = run_noise_experiment(
                X_train, y_train, X_test, y_test,
                feature_names=list(X.columns),
                p_noise=0.1,
                seed=seed,
            )
            top_feature_1 = summary["top_2_features"][0][0]
            top_feature_2 = summary["top_2_features"][1][0]
            print(
                f"{seed:^4} | "
                f"{summary['train_accuracy']:^14.4f} | "
                f"{summary['test_accuracy']:^13.4f} | "
                f"{summary['depth']:^5} | "
                f"{top_feature_1:^13} | "
                f"{top_feature_2:^13}"
            )

        # Q8: Depth vs accuracy plot
        depths = list(range(3, 11))
        depth_values, train_accs, test_accs = depth_accuracy_curve(
            X_train,
            y_train,
            X_test,
            y_test,
            depth_values=depths,
        )

        plt.figure(figsize=(10, 5))
        plt.plot(depth_values, train_accs, marker="o", label="Train Accuracy")
        plt.plot(depth_values, test_accs, marker="s", label="Test Accuracy")
        plt.xlabel("max_depth")
        plt.ylabel("Accuracy")
        plt.title("Decision Tree Accuracy vs max_depth")
        plt.xticks(depth_values)
        plt.grid(alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_dir / "accuracy_vs_depth.png")
        plt.close()

        print("\nLab 5 script executed successfully.")

    except NotImplementedError as exc:
        print("\nA required to do function is still incomplete.")
        print(f"Details: {exc}")
    except FileNotFoundError:
        print("\nDataset file not found.")
        print(f"Expected path: {dataset_path}")


if __name__ == "__main__":
    main()

